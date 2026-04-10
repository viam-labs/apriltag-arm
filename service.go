package apriltagarm

import (
	"context"
	"fmt"
	"strconv"
	"sync"

	vmodutils "github.com/erh/vmodutils"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/rdk/utils"
)

// Model is the Viam model triple for this service.
var Model = resource.NewModel("viam-labs", "service", "apriltag-arm")

func init() {
	resource.RegisterService(
		generic.API,
		Model,
		resource.Registration[generic.Service, *Config]{
			Constructor: newApriltagArmService,
		},
	)
}

type apriltagArmService struct {
	resource.AlwaysRebuild
	resource.TriviallyCloseable

	name   resource.Name
	cfg    *Config
	logger logging.Logger
	mu     sync.Mutex

	tracker   posetracker.PoseTracker
	armComp   arm.Arm
	motionSvc motion.Service
	fsSvc     framesystem.Service
}

func newApriltagArmService(ctx context.Context, deps resource.Dependencies, config resource.Config, logger logging.Logger) (generic.Service, error) {
	cfg, err := resource.NativeConfig[*Config](config)
	if err != nil {
		return nil, err
	}

	svc := &apriltagArmService{
		name:   config.ResourceName(),
		cfg:    cfg,
		logger: logger,
	}

	svc.tracker, err = posetracker.FromDependencies(deps, cfg.CameraName)
	if err != nil {
		return nil, fmt.Errorf("failed to get pose tracker %q: %w", cfg.CameraName, err)
	}

	svc.armComp, err = arm.FromProvider(deps, cfg.ArmName)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm %q: %w", cfg.ArmName, err)
	}

	svc.motionSvc, err = motion.FromProvider(deps, cfg.MotionServiceName)
	if err != nil {
		return nil, fmt.Errorf("failed to get motion service %q: %w", cfg.MotionServiceName, err)
	}

	svc.fsSvc, err = framesystem.FromDependencies(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get frame system service: %w", err)
	}

	return svc, nil
}

func (s *apriltagArmService) Name() resource.Name {
	return s.name
}

func (s *apriltagArmService) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["command"].(string)
	if !ok || command == "" {
		return nil, fmt.Errorf("missing or invalid 'command' field")
	}

	switch command {
	case "save_pose":
		return s.handleSavePose(ctx, cmd)
	case "delete_pose":
		return s.handleDeletePose(ctx, cmd)
	case "move_to_pose":
		return s.handleMoveToPose(ctx, cmd)
	default:
		return nil, fmt.Errorf("unknown command %q; valid commands: save_pose, delete_pose, move_to_pose", command)
	}
}

func (s *apriltagArmService) handleSavePose(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, ok := cmd["name"].(string)
	if !ok || name == "" {
		return nil, fmt.Errorf("save_pose requires a non-empty 'name' field")
	}

	// JSON numbers are float64
	tagIDFloat, ok := cmd["tag_id"].(float64)
	if !ok {
		return nil, fmt.Errorf("save_pose requires a numeric 'tag_id' field")
	}
	tagID := int(tagIDFloat)

	s.mu.Lock()
	defer s.mu.Unlock()

	if _, exists := s.cfg.SavedPoses[name]; exists {
		return nil, fmt.Errorf("pose %q already exists; delete it first", name)
	}
	for existingName, p := range s.cfg.SavedPoses {
		if p.TagID == tagID {
			return nil, fmt.Errorf("tag_id %d is already used by pose %q; delete it first", tagID, existingName)
		}
	}

	tagWorld, err := s.getTagPoseInWorld(ctx, tagID)
	if err != nil {
		return nil, err
	}

	armWorld, err := s.fsSvc.GetPose(ctx, s.cfg.ArmName, "world", nil, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm EEF pose: %w", err)
	}

	// offset = inv(tagWorld) * armWorld
	offset := spatialmath.PoseBetween(tagWorld.Pose(), armWorld.Pose())

	saved := SavedPose{
		TagID:       tagID,
		Point:       offset.Point(),
		Orientation: *offset.Orientation().OrientationVectorDegrees(),
	}

	if s.cfg.SavedPoses == nil {
		s.cfg.SavedPoses = make(map[string]SavedPose)
	}
	s.cfg.SavedPoses[name] = saved

	if err := s.persistConfig(ctx); err != nil {
		delete(s.cfg.SavedPoses, name)
		return nil, fmt.Errorf("failed to persist config: %w", err)
	}

	s.logger.Infof("saved pose %q for tag %d", name, tagID)
	return map[string]interface{}{"success": true, "name": name, "tag_id": tagID}, nil
}

func (s *apriltagArmService) handleDeletePose(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	nameVal := cmd["name"]
	tagIDVal := cmd["tag_id"]

	if (nameVal == nil) == (tagIDVal == nil) {
		return nil, fmt.Errorf("delete_pose requires exactly one of 'name' or 'tag_id'")
	}

	s.mu.Lock()
	defer s.mu.Unlock()

	var targetName string

	if nameVal != nil {
		name, ok := nameVal.(string)
		if !ok || name == "" {
			return nil, fmt.Errorf("'name' must be a non-empty string")
		}
		if _, exists := s.cfg.SavedPoses[name]; !exists {
			return nil, fmt.Errorf("no pose named %q", name)
		}
		targetName = name
	} else {
		tagIDFloat, ok := tagIDVal.(float64)
		if !ok {
			return nil, fmt.Errorf("'tag_id' must be a number")
		}
		tagID := int(tagIDFloat)
		for n, p := range s.cfg.SavedPoses {
			if p.TagID == tagID {
				targetName = n
				break
			}
		}
		if targetName == "" {
			return nil, fmt.Errorf("no pose with tag_id %d", int(tagIDFloat))
		}
	}

	deleted := s.cfg.SavedPoses[targetName]
	delete(s.cfg.SavedPoses, targetName)

	if err := s.persistConfig(ctx); err != nil {
		s.cfg.SavedPoses[targetName] = deleted
		return nil, fmt.Errorf("failed to persist config: %w", err)
	}

	s.logger.Infof("deleted pose %q (tag %d)", targetName, deleted.TagID)
	return map[string]interface{}{"success": true, "name": targetName, "tag_id": deleted.TagID}, nil
}

func (s *apriltagArmService) handleMoveToPose(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	name, ok := cmd["name"].(string)
	if !ok || name == "" {
		return nil, fmt.Errorf("move_to_pose requires a non-empty 'name' field")
	}

	s.mu.Lock()
	saved, exists := s.cfg.SavedPoses[name]
	s.mu.Unlock()

	if !exists {
		return nil, fmt.Errorf("no pose named %q", name)
	}

	tagWorld, err := s.getTagPoseInWorld(ctx, saved.TagID)
	if err != nil {
		return nil, err
	}

	// target = tagWorld * offset
	offsetPose := spatialmath.NewPose(saved.Point, &saved.Orientation)
	target := spatialmath.Compose(tagWorld.Pose(), offsetPose)

	_, err = s.motionSvc.Move(ctx, motion.MoveReq{
		ComponentName: arm.Named(s.cfg.ArmName).String(),
		Destination:   referenceframe.NewPoseInFrame("world", target),
	})
	if err != nil {
		return nil, fmt.Errorf("motion failed: %w", err)
	}

	s.logger.Infof("moved to pose %q (tag %d)", name, saved.TagID)
	return map[string]interface{}{"success": true, "name": name}, nil
}

// getTagPoseInWorld returns the named tag's pose in world frame.
// Returns an error immediately if the tag is not visible.
func (s *apriltagArmService) getTagPoseInWorld(ctx context.Context, tagID int) (*referenceframe.PoseInFrame, error) {
	poses, err := s.tracker.Poses(ctx, []string{}, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get tag poses: %w", err)
	}

	tagPose, ok := poses[strconv.Itoa(tagID)]
	if !ok {
		return nil, fmt.Errorf("tag %d is not visible", tagID)
	}

	tagWorld, err := s.fsSvc.TransformPose(ctx, tagPose, "world", nil)
	if err != nil {
		return nil, fmt.Errorf("failed to transform tag %d pose to world frame: %w", tagID, err)
	}

	return tagWorld, nil
}

// persistConfig writes the current in-memory config back to the Viam cloud config.
func (s *apriltagArmService) persistConfig(ctx context.Context) error {
	attrMap := utils.AttributeMap{
		"camera_name":         s.cfg.CameraName,
		"arm_name":            s.cfg.ArmName,
		"motion_service_name": s.cfg.MotionServiceName,
		"saved_poses":         savedPosesToAttrMap(s.cfg.SavedPoses),
	}
	return vmodutils.UpdateComponentCloudAttributesFromModuleEnv(ctx, s.name, attrMap, s.logger)
}

func savedPosesToAttrMap(poses map[string]SavedPose) map[string]interface{} {
	out := make(map[string]interface{}, len(poses))
	for name, p := range poses {
		out[name] = map[string]interface{}{
			"tag_id":      p.TagID,
			"point":       p.Point,
			"orientation": p.Orientation,
		}
	}
	return out
}

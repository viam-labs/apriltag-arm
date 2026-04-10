package apriltagarm

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// Config holds the module configuration as stored in the Viam machine config.
type Config struct {
	PoseTrackerName   string               `json:"pose_tracker_name"`
	ArmName           string               `json:"arm_name"`
	MotionServiceName string               `json:"motion_service_name"`
	SavedPoses        map[string]SavedPose `json:"saved_poses"`
}

// SavedPose stores an arm end-effector pose as an offset relative to an AprilTag.
// Point and Orientation together describe the transform from the tag's world-frame
// pose to the desired arm EEF world-frame pose.
type SavedPose struct {
	TagID       int                                  `json:"tag_id"`
	Point       r3.Vector                            `json:"point"`
	Orientation spatialmath.OrientationVectorDegrees `json:"orientation"`
}

// Validate checks required fields and returns resource dependency names.
func (c *Config) Validate(path string) ([]string, []string, error) {
	if c.PoseTrackerName == "" {
		return nil, nil, fmt.Errorf("pose_tracker_name is required")
	}
	if c.ArmName == "" {
		return nil, nil, fmt.Errorf("arm_name is required")
	}
	if c.MotionServiceName == "" {
		c.MotionServiceName = "builtin"
	}

	deps := []string{
		posetracker.Named(c.PoseTrackerName).String(),
		motion.Named(c.MotionServiceName).String(),
	}

	return deps, nil, nil
}

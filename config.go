package apriltagarm

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/posetracker"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

// Config holds the module configuration as stored in the Viam machine config.
type Config struct {
	PoseTrackerName   string               `json:"pose_tracker_name"`
	ArmName           string               `json:"arm_name"`
	MotionServiceName string               `json:"motion_service_name"`
	CameraName        string               `json:"camera_name,omitempty"`
	SavedPoses        map[string]SavedPose `json:"saved_poses"`
}

// PixelPoint is a 2-D image coordinate in pixels.
type PixelPoint struct {
	X float64 `json:"x"`
	Y float64 `json:"y"`
}

// SavedPose stores an arm end-effector pose as an offset relative to an AprilTag.
// Point and Orientation together describe the transform from the tag's world-frame
// pose to the desired arm EEF world-frame pose.
// TagPixelCenter is the projected image coordinate of the tag centre at save time
// (populated only when camera_name is configured).
type SavedPose struct {
	TagID          int                                  `json:"tag_id"`
	Point          r3.Vector                            `json:"point"`
	Orientation    spatialmath.OrientationVectorDegrees `json:"orientation"`
	TagPixelCenter *PixelPoint                          `json:"tag_pixel_center,omitempty"`
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
	if c.CameraName != "" {
		deps = append(deps, camera.Named(c.CameraName).String())
	}

	return deps, nil, nil
}

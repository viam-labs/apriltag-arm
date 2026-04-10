# apriltag-arm: Implementation Plan

## Overview

A Viam generic service module written in Go that allows a robot arm to move to
pre-taught positions defined relative to AprilTag fiducial markers. The operator
teaches the module a pose by physically placing the arm at the desired position
while a tag is visible; the module stores the arm's end-effector offset relative
to that tag. At runtime, the module locates the tag in the scene and commands the
arm to the saved pose via the motion service.

---

## Dependencies

| Resource | Type | Purpose |
|---|---|---|
| apriltag component | `PoseTracker` | Detect tags and return their pose in camera frame |
| arm | `Arm` | Source of current EEF pose during save |
| motion service | `motion.Service` | Command arm to target pose |
| frame system service | `framesystem.Service` | Transform tag pose from camera frame to world frame |

---

## Module Config

```json
{
  "camera_name": "camera-1",
  "arm_name": "arm-1",
  "motion_service_name": "builtin",
  "saved_poses": {
    "grasp": {
      "tag_id": 0,
      "offset": {
        "x": 0.0, "y": 0.0, "z": 0.0,
        "o_x": 0.0, "o_y": 0.0, "o_z": 1.0, "theta": 0.0
      }
    }
  }
}
```

`saved_poses` is managed by the module itself — operators do not edit it manually.
It is written back to the machine's cloud config via the Viam app API using the
`erh/vmodutils` `UpdateComponentCloudAttributesFromModuleEnv()` pattern, which
reads credentials and part ID from environment variables injected by viam-server.
No API keys are required in the module config.

---

## DoCommand API

### `save_pose`

Saves the arm's current end-effector pose relative to the specified tag.
Fails if the tag is not currently visible, if `name` is already used by an
existing pose, or if `tag_id` is already associated with another pose.

**Request:**
```json
{ "command": "save_pose", "name": "grasp", "tag_id": 0 }
```

**Response:**
```json
{ "success": true, "name": "grasp", "tag_id": 0 }
```

**Steps:**
1. Fail if `name` already exists in saved poses
2. Fail if `tag_id` is already referenced by any saved pose
3. Call `get_poses()` on the apriltag PoseTracker
4. Fail if `tag_id` is not present in the result
5. Get the tag's pose in world frame via the frame system
6. Get the arm EEF pose in world frame via the frame system
7. Compute offset: `inv(tag_world) * arm_eef_world`
8. Store offset in memory under `name`
9. Write updated `saved_poses` back to cloud config via vmodutils

### `delete_pose`

Removes a saved pose by name or by tag ID. Exactly one of `name` or `tag_id`
must be provided.

**Request (by name):**
```json
{ "command": "delete_pose", "name": "grasp" }
```

**Request (by tag ID):**
```json
{ "command": "delete_pose", "tag_id": 0 }
```

**Response:**
```json
{ "success": true, "name": "grasp", "tag_id": 0 }
```

**Steps:**
1. Fail if both or neither of `name` / `tag_id` are provided
2. Locate the matching pose record (by name or by scanning tag IDs)
3. Fail if no matching record is found
4. Remove the record from memory
5. Write updated `saved_poses` back to cloud config via vmodutils

### `move_to_pose`

Moves the arm to a previously saved pose by locating the associated tag live.
Fails immediately if the tag is not currently visible.

**Request:**
```json
{ "command": "move_to_pose", "name": "grasp" }
```

**Response:**
```json
{ "success": true, "name": "grasp" }
```

**Steps:**
1. Look up `name` in saved poses — fail if not found
2. Call `get_poses()` on the apriltag PoseTracker
3. Fail if the saved pose's `tag_id` is not present in the result
4. Get the tag's current pose in world frame via the frame system
5. Compute target: `tag_world * stored_offset`
6. Call `motion_service.Move(arm, target, world_frame)`

---

## Pose Math

```
# Save
tag_camera  = apriltag.get_poses()[tag_id]
tag_world   = frame_system.Transform(tag_camera, camera_name, world)
arm_world   = frame_system.GetPose(arm_name, world)
offset      = inv(tag_world) * arm_world   // stored in saved_poses

# Move
tag_camera  = apriltag.get_poses()[tag_id]   // live
tag_world   = frame_system.Transform(tag_camera, camera_name, world)
target      = tag_world * offset
motion_svc.Move(arm, PoseInFrame{target, world})
```

---

## File Structure

```
apriltag-arm/
├── main.go          # Module entry point, resource registration
├── service.go       # Service struct, Reconfigure, DoCommand
├── config.go        # Config struct, attribute parsing, validation
├── poses.go         # Pose math helpers (save, compute target)
├── go.mod
├── go.sum
└── doc/
    └── plan.md
```

---

## Key Go Packages

| Package | Use |
|---|---|
| `go.viam.com/rdk/services/generic` | Base service type |
| `go.viam.com/rdk/components/arm` | Arm interface |
| `go.viam.com/rdk/components/posetracker` | AprilTag interface |
| `go.viam.com/rdk/services/motion` | Move commands |
| `go.viam.com/rdk/services/framesystem` | Frame transforms |
| `go.viam.com/rdk/spatialmath` | Pose / orientation math |
| `github.com/erh/vmodutils` | Cloud config write-back |

---

## Error Handling

- `save_pose`: name already exists → return error
- `save_pose`: tag_id already in use → return error
- `save_pose`: tag not visible → return error immediately
- `delete_pose`: both or neither of name/tag_id provided → return error
- `delete_pose`: no matching record → return error
- `move_to_pose`: pose name not found → return error
- `move_to_pose`: tag not visible → return error immediately (no retry/timeout)
- Any command: cloud config write failure → return error (no partial commit)

---

## Reconfigure Behavior

On `Reconfigure`, the module resolves and holds references to all declared
dependencies. If any named resource (arm, camera, motion service, frame system)
cannot be found, `Reconfigure` returns an error and the module is unavailable
until config is corrected. `saved_poses` from attributes are loaded into memory.

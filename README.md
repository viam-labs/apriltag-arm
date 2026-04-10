# apriltag-arm

A Viam generic service module that allows a robot arm to move to pre-taught
positions defined relative to AprilTag fiducial markers.

## Models

- `viam-labs:service:apriltag-arm` — teach and replay arm poses keyed to AprilTag IDs

## DoCommand API

| Command | Required fields | Description |
|---|---|---|
| `save_pose` | `name`, `tag_id` | Save current arm EEF pose relative to visible tag |
| `delete_pose` | `name` or `tag_id` | Remove a saved pose |
| `move_to_pose` | `name` | Move arm to saved pose using live tag detection |

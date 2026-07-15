# clean_table

A modular ROS2 (Humble) pick-and-place behavior for a Tiago robot: navigate to the location of the pick table , Align to the table using the pick table april tag marker , find objects to grasp , grasp each one, carry it to a drop table, and place it , repeating until the pick table is clear.

![Pick and drop schematic](/pick_and_drop_schematic.png)

*The robot aligns to the pick table (markers visible on the object and
table), grasps an object, and carries it along the path shown to the
drop table.*

## Packages

| Package | Build type | Contents |
|---|---|---|
| `clean_table` | `ament_python` | All five behavior nodes + launch file |
| `custom_actions` | `ament_cmake` | Shared action/service definitions |

## Architecture

```
                     ┌────────────────┐
                     │  orchestrator   │  state machine only
                     └───────┬────────┘
        ┌───────────┬────────┼────────────┬───────────────┐
        ▼           ▼        ▼             ▼               ▼
   Nav2's       drive_       align_to_   marker_          pick_place
NavigateToPose  distance_    marker_     lookup_          _server
 (external)     server       server      service
```



### Nodes

| Executable | Node class | Role |
|---|---|---|
| `orchestrator` | `Orchestrator` | State machine tracking task progress; calls out to everything below. |
| `drive_distance_server` | `DriveDistanceServer` | Action server: drives straight forward/backward by a target distance, measured off the change in the middle `/scan_raw` beam. |
| `align_to_marker_server` | `AlignToMarkerServer` | Action server: rotates in place until facing a given AprilTag marker within a yaw tolerance, using `get_marker_transform` for the marker's pose. |
| `marker_lookup_service` | `MarkerLookupService` | Owns the TF buffer/listener and the `/detections` subscription; answers "where is marker X" on demand and publishes visible marker ids. |
| `pick_place_server` | `PickPlaceServer` | Action server wrapping every MoveIt2/gripper interaction: adds/removes collision boxes, opens/closes the gripper, moves the arm, and drives via `drive_distance` — a single goal performs the entire pick or place sequence. |

### Interfaces (`custom_actions`)

| Interface | Type | Used by |
|---|---|---|
| `PickPlace.action` | action | client: `orchestrator`; server: `pick_place_server` |
| `DriveDistance.action` | action | clients: `orchestrator`, `pick_place_server`; server: `drive_distance_server` |
| `AlignToMarker.action` | action | client: `orchestrator`; server: `align_to_marker_server` |
| `GetMarkerTransform.srv` | service | clients: `orchestrator`, `align_to_marker_server`; server: `marker_lookup_service` |



## Build

```bash
cd ros2_ws
colcon build --packages-select custom_actions clean_table
source install/setup.bash
```

## Run

```bash
ros2 launch clean_table clean_table.launch.py
```





To the [Main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

Back to package [README](https://github.com/slgrobotics/slg_bt_plugins/blob/main/README.md)

-----------------------------

## Testing Seggy’s Behavior Tree in Simulation

Publish synthetic sensor messages to `/bt/face_gesture_detect` using `sensor_msgs/msg/Illuminance`.

The BT plugins interpret these fields as:

| Field | Meaning |
|---|---|
| `illuminance` | Face detected when greater than `0.5` |
| `variance` | Face yaw error in radians |
| `header.frame_id` | Gesture: `STOP`, `LIKE`, `OK`, `YES`, `SIX`, or empty |
| `header.stamp` | Message timestamp; must use simulation time |

### Preparation

Start Seggy’s simulation, then open a new terminal:

```bash
source /opt/ros/lyrical/setup.bash
source ~/robot_ws/install/setup.bash
```

Select a **Nav2 Goal** in RViz to start executing `face_aware_nav.xml`.

Run **one sensor publisher at a time**. Press **Ctrl+C** before switching scenarios. Disable any real sensor publisher on the same topic to avoid conflicting inputs.

The commands below use `stamp: now` and `--use-sim-time` so each message receives a fresh simulation timestamp.

### 1. Detect a Face

```bash
ros2 topic pub /bt/face_gesture_detect sensor_msgs/msg/Illuminance \
  "{header: {stamp: now, frame_id: ''}, illuminance: 1.0, variance: 0.4}" \
  --rate 10 --use-sim-time --qos-reliability best_effort
```

Expected behavior:

- Navigation pauses.
- Seggy turns with positive angular velocity.
- Changing `variance` to `-0.4` tests the opposite direction.

> A fixed yaw error continuously commands rotation. The error does not decrease automatically as the simulated robot turns.

### 2. Simulate Successful Alignment

```bash
ros2 topic pub /bt/face_gesture_detect sensor_msgs/msg/Illuminance \
  "{header: {stamp: now, frame_id: ''}, illuminance: 1.0, variance: 0.0}" \
  --rate 10 --use-sim-time --qos-reliability best_effort
```

Expected behavior:

- Turning stops.
- Navigation remains paused while the face is detected.
- Alignment does not complete the navigation goal.

### 3. Remove the Face

```bash
ros2 topic pub /bt/face_gesture_detect sensor_msgs/msg/Illuminance \
  "{header: {stamp: now, frame_id: ''}, illuminance: 0.0, variance: 0.0}" \
  --rate 10 --use-sim-time --qos-reliability best_effort
```

Expected behavior: navigation resumes toward the original goal.

Alternatively, stop publishing. The default perception pump clears stale detections after approximately **one second of simulation time**.

### 4. Send a STOP Gesture

```bash
ros2 topic pub /bt/face_gesture_detect sensor_msgs/msg/Illuminance \
  "{header: {stamp: now, frame_id: STOP}, illuminance: 0.0, variance: 0.0}" \
  --rate 10 --use-sim-time --qos-reliability best_effort
```

Expected behavior:

- STOP takes priority over navigation and face alignment.
- The navigation goal is **canceled**.
- Clearing STOP does not resume the canceled goal.

Stop the STOP publisher before sending a new navigation goal.

### Monitor the Result

Watch navigation action status in another terminal:

```bash
ros2 topic echo /navigate_to_pose/_action/status
```

Status **`5`** means **CANCELED**.

Groot2 can also monitor Seggy’s NavigateToPose tree on port **1667**, as configured in `nav2_params.yaml`.

### Differences Between Trees

| Tree | Face detection | STOP gesture |
|---|---|---|
| `face_aware_nav.xml` | Pauses navigation and aligns with the face | Cancels the goal |
| `minimal_custom_nav.xml` | Pauses navigation without turning | Cancels the goal |
| `follow_point.xml` | No face handling | No gesture handling |

These instructions assume Seggy’s default, pump-based plugin build and an empty ROS namespace.

--------------------------------------

To the [Main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

Back to package [README](https://github.com/slgrobotics/slg_bt_plugins/blob/main/README.md)



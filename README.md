## slg_bt_plugins - a ROS2 package

ROS2 Behavior Trees plugins for different topics

Currently works with [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor)

Please refer to [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

---------------------

**Data source** This package relies on */bt/face_gesture_detect* topic published by [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor)

**Data Dropouts:** If the last message received is older than a specific threshold (e.g., 0.5s),
the condition nodes should automatically return FAILURE
to ensure the robot reverts to its default navigation state instead of "freezing" on old data.

**Data Pump vs. Topic subscriptions** There is a "USE_RCLCPP_SUBSCRIPTIONS" compile condition in CMakeLists.txt, off by default:
- When defined, all BT plugins subscribe individually to the */bt/face_gesture_detect* topic and process data internally (not recommended).
- When not defined, a single *FgsTopicToBlackboard* node is responsible for subscription and data preprocessing. It puts all data on the BT Blackboard. This simplifies code for other plugins.

### Sample behavior trees

There are several BTs in the "*behavior_trees*" directory. 

Here is and example of Groot2 screen, while monitoring *behavior_trees/nav_to_stop.xml* tree:

<img width="1881" height="1331" alt="Screenshot from 2025-12-21 14-21-43" src="https://github.com/user-attachments/assets/2c6baad1-e4ab-4ef0-8c8e-eb1e38caa183" />

### Summary of Behavioral Logic (example)

```
Scenario	    IsStopGesture	    IsFaceDetected	   Resulting Behavior
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Active Stop 	 SUCCESS	          (Ignored)	       Stops Robot (CancelControl)
 Gesture

Face Seen,	     FAILURE	          SUCCESS	         Turns toward person (TurnTowardFace)
 No Stop

No Face, 	     FAILURE	          FAILURE	         Normal Navigation (NavigateToPose)
 No Stop

Data Streams	 FAILURE	          FAILURE	         Normal Navigation (Safe default)
 Cease
```

---------------------

[main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

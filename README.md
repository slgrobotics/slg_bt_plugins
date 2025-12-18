## slg_bt_plugins - a ROS2 package

ROS2 Behavior Trees plugins for different topics

Works with [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor)

Please refer to [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

---------------------

### Summary of Behavioral Logic

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

**Data Dropouts:** If the last message received is older than a specific threshold (e.g., 0.5s),
the condition nodes should automatically return FAILURE
to ensure the robot reverts to its default navigation state instead of "freezing" on old data.

---------------------

[main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

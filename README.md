# ROS2/Nav2 Behavior Trees: Custom plugins package

ROS2 Behavior Trees plugins for bringing different ROS2 topics in the scope of BTs and other tasks.

Currently works with [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor) and also supplies [BT Nodes for general use](#additional-general-purpose-bt-nodes).

Please refer to [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor) (*Seggy* robot)

---------------------

## BT Nodes for DFRobot *AI Gesture and Face Detection Sensor*

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

A more advanced and fully functional tree is described [here](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor#example-behavior-tree).

--------------------

## Additional General Purpose BT nodes

### Typed Blackboard Utility Nodes

This document describes the **strongly-typed Blackboard helper nodes** provided in `slg_bt_plugins`.
These nodes are designed to make Behavior Trees easier to read, safer to maintain, and more expressive,
especially when implementing **latches**, **state flags**, and **symbolic decisions** (such as gestures).

All nodes follow the conventions used throughout `slg_bt_plugins`:

- Access the ROS node via the BT blackboard (`"node"`)
- Use throttled ROS logging
- Deterministic SUCCESS / FAILURE semantics
- No implicit string-to-type conversions

## Contents

1. Conditions
  - [`IsTrue`](#istrue)
  - [`IsFalse`](#isfalse)
  - [`IsBlackboardStringEqual`](#isblackboardstringequal)
2. Actions (Blackboard setters)
  - [`SetBlackboardBool`](#setblackboardbool)
  - [`SetBlackboardInt`](#setblackboardint)
  - [`SetBlackboardDouble`](#setblackboarddouble)
  - [`SetBlackboardString`](#setblackboardstring)
3. [Common Behavior Tree Patterns](https://chatgpt.com/s/t_694b308d7e048191a74ca1d165d6d1b0) [ChatGPT.com generated]

---------------

## 1. Conditions

### IsTrue

Returns **SUCCESS** if a boolean input value is `true`.

This node is typically used to:
- check latch flags
- guard execution paths
- improve readability compared to `Inverter`-based logic

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `key` | `bool` | Boolean value (usually from blackboard via `{}`) |

#### Behavior

- SUCCESS if `key == true`
- FAILURE if `key == false` or missing

#### Example

```
<IsTrue key="{face_pause_active}"/>
```

### IsFalse

**Condition node** that returns **SUCCESS** when a boolean input value is `false`.

Useful for:
- clearing latches
- expressing “not detected” conditions
- simplifying inverted logic

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `key` | `bool` | Boolean value (usually from blackboard via `{}`) |

#### Behavior

- SUCCESS if `key == false`
- FAILURE if `key == true` or missing

#### Example

```
<IsFalse key="{is_face_detected}"/>
```

### IsBlackboardStringEqual

**Condition node** that compares a string stored on the blackboard to a **literal string**.

Unlike most BT nodes, the `key` port refers to a **blackboard key name**, not a `{}` substitution.
This makes intent explicit and avoids ambiguity.

Typical use cases:
- gesture recognition (`STOP`, `OK`, `YES`)
- mode or state matching
- symbolic branching in BTs

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `key` | `string` | Blackboard key name (e.g. `"gesture"`) |
| `compare_to_string` | `string` | Literal string to compare against |

#### Behavior

- SUCCESS if `blackboard[key] == compare_to_string`
- FAILURE otherwise (including missing key)

#### Example

```
<IsBlackboardStringEqual key="gesture" compare_to_string="STOP"/>
```

## 2. Actions (Blackboard setters)

### SetBlackboardBool

**Action node** that writes a boolean value into the Behavior Tree blackboard.

Common uses:
- latch flags (e.g. `face_pause_active`)
- STOP / resume states
- feature enable/disable toggles

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `output_key` | `string` | Blackboard key name to write |
| `value` | `bool` | Boolean value |

#### Behavior

- Writes the value to the blackboard
- Returns SUCCESS on success

#### Example

```
<SetBlackboardBool output_key="face_pause_active" value="true"/>
```

### SetBlackboardInt

**Action node** that writes an integer value into the Behavior Tree blackboard.

Useful for:
- counters
- retry counts
- discrete mode identifiers

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `output_key` | `string` | Blackboard key name to write |
| `value` | `int` | Integer value |

#### Behavior

- Writes the value to the blackboard
- Returns SUCCESS on success

#### Example

```
<SetBlackboardInt output_key="retry_count" value="3"/>
```

### SetBlackboardDouble

**Action node** that writes a double-precision floating-point value into the Behavior Tree blackboard.

Typical uses:
- tolerances
- gains
- timing thresholds
- numeric configuration parameters

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `output_key` | `string` | Blackboard key name to write |
| `value` | `double` | Double value |

#### Behavior

- Writes the value to the blackboard
- Returns SUCCESS on success

#### Example

```
<SetBlackboardDouble output_key="yaw_tolerance" value="0.15"/>
```

### SetBlackboardString

**Action node** that writes a string value into the Behavior Tree blackboard.

Typical uses:
- gesture names (`STOP`, `OK`, `YES`)
- state labels
- symbolic modes

#### Ports

| Name | Type | Description |
|-----|------|-------------|
| `output_key` | `string` | Blackboard key name to write |
| `value` | `string` | String value |

#### Behavior

- Writes the value to the blackboard
- Returns SUCCESS on success

#### Example

```
<SetBlackboardString output_key="gesture" value="STOP"/>
```

---------------------

[main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor)

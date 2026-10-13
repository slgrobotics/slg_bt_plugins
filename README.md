# ROS2/Nav2 Behavior Trees: Custom plugins package

ROS2 Behavior Trees plugins for bringing different ROS2 topics in the scope of BTs and other tasks.

Currently works with [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor) and also supplies [BT Nodes for general use](#additional-general-purpose-bt-nodes).

Please refer to [main project Wiki](https://github.com/slgrobotics/articubot_one/wiki/Behavior-Tree-for-Gesture-and-Face-Detection-Sensor) (*Seggy* robot)

> **Note:** changes made to work under Lyrical:
> - Changed the blackboard node type to nav2::LifecycleNode::SharedPtr.
> - Updated ROS factories and dependencies.
> - Added a regression test covering all 13 plugin constructors and a perception-pump tick.

---------------------

> **Tip** This *"main"* branch was updated to work in ROS 2 Lyrical.
> For *Jazzy* and *Kilted* clone it at hash `1afbbf18a95d152f89a9f8e3da2571333d7e77da` (Oct 5,2026 9:25 PM CST):
> ```
> cd ~/robot_ws/src
> git clone https://github.com/slgrobotics/slg_bt_plugins.git
> cd slg_bt_plugins
> git checkout 1afbbf18a95d152f89a9f8e3da2571333d7e77da
> ```

---------------------

## BT Nodes for DFRobot *AI Gesture and Face Detection Sensor*

**Data source** This package relies on */bt/face_gesture_detect* topic published by [DFRobot AI Gesture and Face Detection Sensor package](https://github.com/slgrobotics/face_gesture_sensor)

**Testing: live robot and in simulation** Please review [this guide](https://github.com/slgrobotics/slg_bt_plugins/blob/main/test/README.md).

**Data Dropouts:** If the last message received is older than a specific threshold (e.g., 0.5s),
the condition nodes should automatically return FAILURE
to ensure the robot reverts to its default navigation state instead of "freezing" on old data.

**Data Pump vs. Topic subscriptions** The `USE_RCLCPP_SUBSCRIPTIONS` CMake option is off by default:
- When defined, all BT plugins subscribe individually to the */bt/face_gesture_detect* topic and process data internally (not recommended).
- When not defined, a single *FgsTopicToBlackboard* node is responsible for subscription and data preprocessing. It puts all data on the BT Blackboard. This simplifies code for other plugins.

### ROS 2 Lyrical

This version uses `nav2_ros_common` and expects `nav2::LifecycleNode::SharedPtr`
in the BT blackboard entry `node`, matching Lyrical Nav2. Custom test runners
must also use this type and configure/activate the node before ticking trees
that publish velocity commands. The node factories use Nav2 publishers,
subscriptions, and service clients. This version requires Lyrical; the previous
`rclcpp::Node::SharedPtr` blackboard interface used with Jazzy is incompatible.

### Sample behavior trees

The XML files are installed to `share/slg_bt_plugins/behavior_trees`.

| Tree | Build mode | Purpose |
|------|------------|---------|
| `fgs_data_to_blackboard.xml` | Either | Continuous perception pump |
| `dump_blackboard_data.xml` | Either | Continuous perception pump and logging |
| `minimal_pump.xml` | Default | Continuous pump and face condition tracing |
| `nav_to_stop.xml` | Default | Standalone turning demo; STOP inhibits turning (historical filename) |
| `face_aware_nav.xml` | Default | Navigation with perception pump, STOP cancellation, and face alignment |
| `no_pump/turn_to_human.xml` | Subscriptions | Standalone turning demo; STOP inhibits turning |
| `no_pump/face_aware_nav.xml` | Subscriptions | Navigation with individual subscriptions |

Build subscription mode with:

```bash
colcon build --packages-select slg_bt_plugins --cmake-args -DUSE_RCLCPP_SUBSCRIPTIONS=ON
```

Use `-DUSE_RCLCPP_SUBSCRIPTIONS=OFF` to return to the default mode. The trees
with blackboard input mappings require the default mode; the `no_pump` trees
require subscription mode.

For the default build, select the navigation tree in the BT Navigator parameters:

```yaml
default_nav_to_pose_bt_xml: "$(find-pkg-share slg_bt_plugins)/behavior_trees/face_aware_nav.xml"
plugin_lib_names: ["slg_bt_plugins"]
```

Keep any other custom libraries in `plugin_lib_names`. Lyrical loads its standard
Nav2 BT libraries automatically. The navigation examples include Nav2's installed
`NavigateToPoseWReplanningAndRecovery` subtree with the goal/path blackboard
entries shared through `_autoremap`. They do not send recursive NavigateToPose
goals. The stock subtree expects the usual configured planner/controller IDs.

STOP takes priority over face alignment and cancels the NavigateToPose goal.
Once STOP is detected, the cancellation branch stays selected even if the gesture
clears. It halts active navigation/turning, retries the cancellation service on
failure, and remains RUNNING until Nav2 processes the cancellation. Clearing STOP
does not resume the canceled goal; send a new goal to navigate again. The relative
`navigate_to_pose` action name respects the robot namespace. Cancellation applies
to all goals on that action server, matching the CancelNavToPose plugin's contract.

Face detection still pauses translation and aligns with the face. Alignment stays
RUNNING even after reaching tolerance; navigation resumes when the face disappears.
Only the stock navigation subtree reports normal goal completion or failure.

The diagnostic and standalone turning trees run continuously until explicitly
halted. They are not navigation goal trees.

### Summary of Behavioral Logic (example)

```
Scenario	    IsStopGesture	    IsFaceDetected	   Resulting Behavior
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
Active Stop 	 SUCCESS	          (Ignored)	       Cancels navigation / inhibits turning
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

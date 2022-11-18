# ROS 2 Driver for Generic Joysticks

The joy package contains joy_node, a node that interfaces a generic joystick to ROS 2. This node publishes a "sensor_msgs/msg/Joy" message, which contains the current state of each one of the joystick's buttons and axes.

## Supported Hardware

This node should work with any joystick that is supported by SDL.

## Published Topics

* joy ([sensor_msgs/msg/Joy](http://docs.ros.org/api/sensor_msgs/html/msg/Joy.html)): outputs the joystick state.

## Subscribed Topics
* joy/set_feedback ([sensor_msgs/msg/JoyFeedback](http://docs.ros.org/api/sensor_msgs/html/msg/JoyFeedback.html): Controls the "rumble" device of a joystick, if it has one.

## Parameters

* device_id (int, default: 0)
  * The joystick device to use.

* device_name (string, default: "")
  * The joystick name to use.  This can be useful when multiple different joysticks are attached.  If both device_name and device_id are specified, device_name takes precedence.

* deadzone (double, default: 0.05)
  * Amount by which the joystick has to move before it is considered to be off-center. This parameter is specified relative to an axis normalized between -1 and 1. Thus, 0.1 means that the joystick has to move 10% of the way to the edge of an axis's range before that axis will output a non-zero value.

* autorepeat_rate (double, default: 20.0)
  * Rate in Hz at which a joystick that has a non-changing state will resend the previously sent message.  If set to 0.0, autorepeat will be disabled, meaning joy messages will only be published when the joystick changes.  Cannot be larger than 1000.0.

* sticky_buttons (bool, default: false)
  * Whether buttons are "sticky".  If this is false (the default), then a button press will result in a 1 being output, and a button release will result in a 0 being output.  If this is true, then a button press will toggle the current state of the button, and a button release will do nothing.  Thus, hitting the button while it is currently 0 will switch it to 1, and keep it at 1 until the button is pressed again.

* coalesce_interval_ms (int, default: 1)
  * The number of milliseconds to wait after an axis event before publishing a message.  Since the kernel sends an event for every change, this can significantly reduce the number of messages published.  Setting it to 0 disables this behavior.  The default of 1 ms is a good compromise between message delays and number of messages published.

## Technical note about interfacing with the joystick on Linux

On Linux there are two different ways to interface with a joystick.  The distinction only makes a difference when attempting to pass through the device into a container or virtual machine.  The first interface is via the joystick driver subsystem, which generally shows up as a device in /dev/input/js0 (or other numbers at the end).  This is the way that the "joy_linux" package accesses the joystick.  The second way to interface is through the generic event subsystem, which generally shows up as /dev/input/event7 (or other numbers at the end).  This is the way that SDL (and hence this "joy" package) accesses the joysticks.

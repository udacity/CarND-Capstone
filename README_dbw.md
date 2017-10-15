# DBW Node

This is a README file to explain the **DBW Node(drive-by-wire node)** that I´m working on the **Running-Out-Of-Fuel Team** for Self Driving Car Nanodegree Capstone.

## Subscribe

This node subscribe the topics:

* /current_velocity
* /twist_cmd
* /vehicle/dbw_enabled

## Publish

Publish steering, throttle, and brake commands to:

* /vehicle/steering_cmd
* /vehicle/throttle_cmd
* /vehicle/brake_cmd

# dbw_node.py

DBWNode receives subscribed messages and pass the messages to Controller instance and receive result, finally publishes to vehicle command nodes.

## '__init__' function

1. Define capacities and min/max values .
2. Create publisher nodes for steering, throttle, and brake.
3. Create Controller instance.
4. Set subscriber and callbacks.
5. Start loop for publishing messages.

## 'loop' functino

1. Check wheather dbw is enabled.
2. Pass the current values to controller instance.
3. Publish the messages

## 'publish' function

#### Throttle

1. Create ThrottleCmd instance.
2. Set throttle value.
3. Publish the message to /vehicle/throttle_cmd topic.

#### Steering

1. Create SteeringCmd instance.
2. Set steering angle value.
3. Publish the message to /vehicle/steering_cmd topic.

#### Brake

1. Create BrakeCmd instance.
2. Set brake value.
3. Publish the message to /vehicle/brake_cmd topic.

## 'current_velocity_callback' function

1. Receive the message from /current_velocity topic.
2. Set current value to self.current_velocity for next publish.

## 'twist_cmd_callback' function

1. Receive the message from /twist_cmd topic.
2. Set current value to self.twist_cmd for next publish.

## 'dbw_enabled_callback' function

1. Receive the message from /vehicle/dbw_enabled topic.
2. Set current value to self.dbw_enabled for next publish.

# twist_controller.py

Controller has PID and YawController instances for calculate next movement.

## '__init__' function

Receive maximum values for controling the vehicle. Create PID and YawController instances with received values.

## 'control' function

1. Calculate delta time since last call.
2. Check if dbw is enabled.
3. Calculate next throttle, brake values using PID with received values.
4. Calculate next steering angle using YawController with received values.

# Conclusion

DBWNode will calculate next actions for throttle, brake, and steering angle and publish the values for vehicle control system.

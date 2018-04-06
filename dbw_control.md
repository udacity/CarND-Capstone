# Drive-By-Wire (DBW) Node

The autonomous vehicle for which this project is being developed uses a [Dataspeed](http://dataspeedinc.com/) drive-by-wire (DBW) interface. The interface accepts a variety of control commands, including (but not limited to):

* **throttle** - for pedal position
* **brake** - for braking torque (or pedal position)
* **steering** - for steering wheel angle

The job of the DBW control node in this software is publish appropriate values to these three control command interfaces, based on input from upstream message sources:

* **twist_cmd** - target linear and angular velocity published by the [waypoint follower](https://github.com/team-fusionx/CarND-Capstone/wiki/Waypoint-Follower)
* **current_velocity** - current linear velocity published by the vehicle's sensor system
* **dbw_enabled** - flag indicating if the drive-by-wire system is currently engaged
* **is_decelerating** - flag indicating if the [waypoint updater](https://github.com/team-fusionx/CarND-Capstone/wiki/Waypoint-Updater) is attempting to decelerate the vehicle

## ROS Node

The main ROS node definition for the DBW system exists in [dbw_node.py](ros/src/twist_controller/dbw_node.py).

This node performs the following setup upon being created:

* accepts a number of [vehicle parameters from the vehicle configuration](ros/src/twist_controller/dbw_node.py#L24-L33)
* implements the dbw_node ROS topic [publishers](ros/src/twist_controller/dbw_node.py#L38-L43) and [subscribers](ros/src/twist_controller/dbw_node.py#L62-L76)
  * The four ROS topic subscribers ([twist_cmd](ros/src/twist_controller/dbw_node.py#L120-L123), [current_velocity](ros/src/twist_controller/dbw_node.py#L125-L127), [dbw_enabled](ros/src/twist_controller/dbw_node.py#L110-L115), [is_decelerating](ros/src/twist_controller/dbw_node.py#L117-L118)) assign various instance variables, used by the Control class, once extracted from the topic messages
* creates a [Controller instance](ros/src/twist_controller/dbw_node.py#L59-L60) to manage the specific vehicle control
* enters [a loop which provides the most recent data from topic subscribers to the Controller instance](ros/src/twist_controller/dbw_node.py#L81-L90)

The loop executes at a target rate of 50Hz (any lower than this and the vehicle will automatically disable the DBW control interface for safety). The loop checks [if the DBW node is enabled, and all necessary data is available for the Controller](ros/src/twist_controller/dbw_node.py#L129-L130), then hands the appropriate values (current and target linear velocity, target angular velocity, and whether the vehicle is attempting to decelerate) to the Controller. Once the Controller returns throttle, brake, and steering commands, these are [published on the corresponding ROS interfaces](ros/src/twist_controller/dbw_node.py#L92-L108).

## Controller

A [Controller class](ros/src/twist_controller/twist_controller_mod.py) manages the computation of throttle, brake, and steering control values. The controller has two main components: speed control and steering control.

The Controller, upon [initialization](ros/src/twist_controller/twist_controller_mod.py#L35-L48), sets up a [Yaw Controller instance](https://github.com/team-fusionx/CarND-Capstone/blob/master/ros/src/twist_controller/yaw_controller.py) for computing steering measurements, as well as three [Low Pass Filter instances](ros/src/twist_controller/lowpass.py) for throttle, brake, and steering.

### Speed control

At each control request, the [following steps are performed](ros/src/twist_controller/twist_controller_mod.py#L72-L77):
* Compute the timestep from the last control request to this one
* Compute the linear velocity error (difference between target and current linear velocity)
* Reset PI control integrators if the vehicle is stopped (has a zero target and current linear velocity); more on the integrators later

Next, the raw throttle and brake values are computed. The basic design:
* adds variable throttle if the vehicle is accelerating or vehicle is slowing dowm but not significantly enough to release the throttle entirely
* adds variable braking if the vehicle is travelling too fast relative to the target speed (and simply releasing throttle will not slow down fast enough)
* adds constant braking if the vehicle is slowing down to a stop

[In code, this specifally translates to](ros/src/twist_controller/twist_controller_mod.py#L86-L104):
* If the vehicle is decelerating and the target and current linear velocity are below a stopping threshold (1.5 m/s)
  * Set the raw braking value to 50 Nm
  * Reset the velocity control integrator
* Else if the vehicle is traveling significantly faster than desired (with a buffer which scales linearly with the current linear velocity)
  * Use the braking PI controller to compute a raw braking value based on the negative of the velocity error
  * Reset the velocity control integrator
* Else if the vehicle is accelerating OR the vehicle is traveling signficantly slower than the target speed
  * Use the acceleration PI controller to compute a raw throttle value based on the velocity error
  * Reset the braking control integrator

Once the raw throttle and braking values are computed, the raw braking value is [sent through a low pass filter](ros/src/twist_controller/twist_controller_mod.py#L106-L107) to prevent rapid braking spikes. If the resulting value is too small (below 10Nm), the braking value is reduced to zero; else, the throttle is reduced to zero. This is to prevent the brake and throttle from actuating at the same time. Finally, the throttle value is [sent through a separate low pass filter](ros/src/twist_controller/twist_controller_mod.py#L118-L119) to prevent rapid throttle spikes.

### Steering control

The target linear velocity, target angular velocity, and current linear velocity are [sent into the Yaw controller](ros/src/twist_controller/twist_controller_mod.py#L127). This controller computes a nominal steering angle based on a simple kinematic bicycle model. Finally, this steering value is sent through its own low pass filter to smooth out final steering commands.
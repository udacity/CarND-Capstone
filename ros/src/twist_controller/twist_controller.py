from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement

        self.speed_controller = PID(0.5, 0.02, 0.2)
	self.steering_controller = PID(5, 0.05, 1, -0.5, 0.5)
	

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

	target_linear_vel = args[0]
	target_angular_vel = args[1]
	current_linear_vel = args[2]
	current_angular_vel = args[3]
	dbw_en = args[4]
	dt = args[5]

	linear_vel_cte = (target_linear_vel - current_linear_vel)
	
	#angular_vel_cte = target_angular_vel - current_angular_vel
	angular_vel_cte = target_angular_vel
	# $rostopic echo /twist_cmd says target linear velocity is fixed to 11.
	# but target angular velocity is changing based on vehicle's orientation.

	if dbw_en is False: #PID must not accumulate error under manual control
	    self.speed_controller.reset()
	    self.steering_controller.reset()

	linear_vel = self.speed_controller.step(linear_vel_cte, dt)
	steer = self.steering_controller.step(angular_vel_cte, dt)
	
	
	throttle = 0
	brake = 0
	
	if linear_vel > 0:
	    throttle = linear_vel
	else:
	    brake = abs(linear_vel)*20000

        return throttle, brake, steer

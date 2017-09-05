GAS_DENSITY = 2.858
ONE_MPH = 0.44704

# Import helper classes
from  yaw_controller import YawController
from lowpass import LowPassFilter 
from pid import PID


class Controller(object):
    def __init__(self, throttle, brake, steering):
    	"""
    	:param throttle - PID controller applied to the throttle
    	:param brake - PID controller applied to the brake
    	:param steering - PID controller applied to the steering   
    	"""
        self.throttle = throttle
        self.brake = brake 
        self.steering = steering

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        return 1., 0., 0.

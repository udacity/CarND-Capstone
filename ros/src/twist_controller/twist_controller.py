from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, kp, ki, kd, max_accel, max_decel):
    	self.kp = kp
    	self.ki = ki
    	self.kd = kd
    	self.max_accel = max_accel
    	self.max_decel = max_decel

    	self.pid = PID(self.kp, self.ki, self.kd, mn=self.max_decel, mx=self.max_accel)


    def reset(self):
    	self.pid.reset()
    	

    def control(self, error, sample_time):
        # Return throttle, brake
        brake = 0.0
        throttle = 0.0

        value = self.pid.step(error, sample_time)

        if (value > 0.0):
        	throttle = value
        else:
        	brake = value

        return throttle, brake

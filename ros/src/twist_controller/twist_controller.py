from pid import PID

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    throttle = 0
    brake = 0
    steer = 0
    def __init__(self, *args, **kwargs):
        # TODO Choose proper values for here
        self.pid_speed = PID(4,0.1,0.5,0,1)
        self.pid_brake = PID(8,0.1,0.5,0.1)
        pass
    
    def reset(self):
        self.pid_speed.reset()

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer
        
        for key in kwargs:
            if key is 'error_velocity':
                self.v_err = kwargs[key]
                # Pass in error and time step
                self.throttle = self.pid_speed.step(self.v_err,0.1)
            if key is 'error_brake':
                self.b_err = kwargs[key]
                # Pass in error and time step
                self.brake = self.pid_brake.step(self.b_err,0.1)
        
        return self.throttle, self.brake, 0.

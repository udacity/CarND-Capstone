
GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, accel_limit, decel_limit):
        self.yaw_controller = yaw_controller
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit

    def control(self, current_twist, proposed_twist, dt):
        current_linear = current_twist.linear.x
        proposed_linear = proposed_twist.linear.x
        proposed_angular = proposed_twist.angular.z
        proposed_acc = (proposed_linear - current_linear)/dt
        if proposed_acc > 0:
            throttle = min(proposed_acc, self.accel_limit)/self.accel_limit
            brake = 0.0
        else:
            throttle = 0.0
            brake = min(-proposed_acc, self.decel_limit)/self.decel_limit
        steer = self.yaw_controller.get_steering(proposed_linear, proposed_angular, current_linear)
        return throttle, brake, steer

from pid import PID
import matplotlib.pyplot as plt
import sys
from lowpass import LowPassFilter, SimpleFilter


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, yaw_controller, accel_limit, decel_limit):
        self.yaw_controller = yaw_controller
        self.accel_limit = accel_limit
        self.decel_limit = decel_limit
        # self.pid = PID(5, 0.5, 0.5)
        self.pid = PID(2.5, 0.0005, 0.1, -1.0, 1.0)
        self.errs = []
        self.thr = []
        #self.filter = LowPassFilter(10.0, 1.0)
        self.filter = SimpleFilter(0.95)

    def control(self, current_twist, proposed_twist, dt):
        current_linear = current_twist.linear.x
        proposed_linear = proposed_twist.linear.x
        proposed_angular = proposed_twist.angular.z

        # self.filter.filt(proposed_linear)
        # proposed_linear = self.filter.get()

        velo_err = (proposed_linear - current_linear)

        # self.errs.append(proposed_linear)
        # if len(self.errs) > 2500:
        #     # plt.plot(self.errs[:20])
        #     # plt.show()
        #     # plt.plot(sum(self.errs), "b+")
        #     # plt.show()
        #     plt.plot(self.errs[100:])
        #     plt.show()
        #     plt.plot(self.thr[100:])
        #     plt.show()
        #     sys.exit(0)

        acc_control = self.pid.step(velo_err, dt)
        if acc_control > 0:
            throttle = acc_control
            brake = 0
        else:
            throttle = 0
            brake = -acc_control

        # proposed_acc = velo_err
        # max_acc = self.accel_limit*dt
        # max_dec = self.decel_limit*dt
        # if proposed_acc > 0:
        #     throttle = min(proposed_acc, max_acc)/max_acc
        #     brake = 0.0
        # else:
        #     throttle = 0.0
        #     brake = max(proposed_acc, max_dec)/max_dec

        # self.errs.append(velo_err)
        # if (len(self.errs) > 300):
        #     plt.plot(self.errs[0:])
        #     plt.show()
        #     plt.plot(self.errs[150:])
        #     plt.show()

        steer = self.yaw_controller.get_steering(proposed_linear, proposed_angular, current_linear)
        #steer = self.yaw_controller.steer_ratio * proposed_angular
        return throttle, brake, steer

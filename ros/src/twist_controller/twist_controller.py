from pid import PID
import rospy

GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, pid_kp, pid_ki, pid_kd, min_value, max_value):
        self.pid_controller = PID(pid_kp, pid_ki, pid_kd, mn=min_value, mx=max_value)  # throttle [0-1]
        self.current_timestamp = rospy.Time.now()
        self.previous_timestamp = rospy.Time.now()
        self.pid_is_reset = True
        return

    # note here we don't care about the steering. We only control the throttle
    def control(self, ref_linear_vel, cur_linear_vel, dbw_enabled):
        self.current_timestamp = rospy.Time.now()

        if dbw_enabled:
            error_speed = ref_linear_vel - cur_linear_vel
            sample_time = (self.current_timestamp - self.previous_timestamp).nsecs / 1e9
            if sample_time != 0.0:
                throttle = self.pid_controller.step(error_speed, sample_time)  # pass the time in seconds
                # rospy.loginfo("sample_time = %s error_speed = %s  throttle = %s ref_linear_vel = %s cur_linear_vel = %s",sample_time, error_speed, throttle, ref_linear_vel, cur_linear_vel)
            else:
                throttle = 1.0  # BUG - this is for debug only, should be 0.0
                rospy.logwarn("ERROR: sample_time = %s throttle = %s", sample_time, throttle)

            if self.pid_is_reset:
                self.pid_is_reset = False
                rospy.logwarn("PID control Enabled")
        else:
            self.reset()
            return 0, 0, 0  # Does not matter as it won't be used

        brake = 0
        steering = 0
        self.previous_timestamp = self.current_timestamp
        return throttle, brake, steering

    def reset(self):
        if not self.pid_is_reset:
            self.pid_controller.reset()
            rospy.logwarn("PID control Reset and Stopped")
            self.pid_is_reset = True

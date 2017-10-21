
import rospy
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, kp, ki, kd, mn=MIN_NUM, mx=MAX_NUM):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.min = mn
        self.max = mx

        self.int_val = self.last_int_val = self.last_error = 0.

        #twiddle parameters
        self.p_error = 0
        self.i_error = 0
        self.d_error = 0

    def reset(self):
        self.int_val = 0.0
        self.last_int_val = 0.0

    def step(self, error, sample_time):
        self.last_int_val = self.int_val

        integral = self.int_val + error * sample_time;
        derivative = (error - self.last_error) / sample_time;

        y = self.kp * error + self.ki * self.int_val + self.kd * derivative;
        val = max(self.min, min(y, self.max))

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        
        return val

    def total_error(self, p):
        rospy.loginfo("Calculating errors with the following:  kp =   " + str(p[0]) + " ki = " + str(p[1]) + " kd = " + str(p[2]) )
        rospy.loginfo("Calculating errors with the following:  p_error =   " + str(self.p_error) + " i_error = " + str(self.i_error) + " d_error = " + str(self.d_error) )

        updated_error =  p[0] * self.p_error + p[2] * self.d_error + p[1] * self.i_error;
        rospy.loginfo("Check Error: " + str(float(updated_error)))
        return updated_error

    def twiddle(self, cte):
        tol = 0.2
        dp = [1,1,1]
        p = [0,0,0 ]
        rospy.loginfo("Initial Error: " + str(float(cte)))     

        best_error = cte
        rospy.loginfo("beginning twiddle")
        rospy.loginfo("Starting dp = " + str(float(dp[0]))+ "  " + str(float(dp[1]))+ "  " +str(float(dp[2])))
        while sum(dp) > tol:            
            for i in range(len(p)):
                p[i] += dp[i]
                # in case we cannot get a better value, this will hold the original                
                error = self.total_error(p)
                if error < best_error:
                    rospy.loginfo("error is less")
                    rospy.loginfo("error = " + str(float(cte)))
                    best_error = error
                    dp[i] *= 1.1
                else:
                    rospy.loginfo("error is more")
                    rospy.loginfo("error = " + str(float(cte)))
                    p[i] -= 2 * dp[i]
                    error = self.total_error(p)

                    if error < best_error:
                        rospy.loginfo("embedded if")
                        rospy.loginfo("error = " + str(float(cte)))
                        best_error = error
                        dp[i] *= 1.1
                    else:
                        rospy.loginfo("embedded else")
                        rospy.loginfo("error = " + str(float(cte)))
                        p[i] += dp[i]
                        dp[i] *= .09

                rospy.loginfo("dp = " + str(float(dp[0]))+ "  " + str(float(dp[1]))+ "  " +str(float(dp[2])))
        rospy.loginfo("kp = " + str(dp[0]) + " ki = " + str(dp[1]) + " kd = " + str(dp[2]) )
        return p[0], p[1], p[2]

    def updateError(self, cte):
        rospy.loginfo("--------------------------------------------------------------------------------------------------------------------------------")
        rospy.loginfo("Error from step method is: " + str(float(cte)))
        self.d_error = cte - self.p_error
        self.p_error = cte
        self.i_error += cte

        self.kp, self.ki, self.kd = self.twiddle(cte)

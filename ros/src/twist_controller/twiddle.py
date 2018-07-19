import rospy
import pid
import numpy as np


class Twiddle(pid.PID):

    def __init__(self, coeffs,
                 mn=pid.MIN_NUM, mx=pid.MAX_NUM,
                 delta_coeffs=None,
                 active=False):
        super(Twiddle, self).__init__(coeffs, mn, mx)

        # Tuning delta for each coefficient
        self.delta_coeffs = delta_coeffs if delta_coeffs is not None else [coeff * 0.1 for coeff in coeffs]
        self.coeff_idx = 0
        self.active = active
        self.lowest_error = np.inf
        self.tuning_count = 0
        self.state_machine = StateMachine(StateMachine.init)

        self.reset()

    def reset(self):
        super(Twiddle, self).reset()
        self.accumulated_error = 0.0

    def step(self, error, sample_time):
        """Calculate the score for the twiddle algorithm

        Larger errors are punished more by taking the square.
        Negative errors, that is when the actual value is above the target value, is punished by an extra factor of 10.
        """
        if error > 0.0:
            # Actual value is below target value.
            self.accumulated_error += (error**2.0)
        else:
            # Actual value is above target value.
            self.accumulated_error += 10.0 * (error**2.0)
        return super(Twiddle, self).step(error, sample_time)

    def set_next_params(self):
        if self.active:
            if self.accumulated_error < self.lowest_error:
                # This tuning is the best so far.
                self.lowest_error = self.accumulated_error
                rospy.loginfo("%s Improvement %s coeffs %s delta %s",
                              self.tuning_count, self.accumulated_error, self.coeffs, self.delta_coeffs)

                if isinstance(self.state_machine.state, Init):
                    # Try a more aggressive tuning next time.
                    self.delta_coeffs[self.coeff_idx] *= 1.1
                    self.state_machine.state = StateMachine.increase
            else:
                rospy.loginfo("%s No improvement %s coeffs %s delta %s",
                              self.tuning_count, self.accumulated_error, self.coeffs, self.delta_coeffs)
            self.state_machine.run(self)
            self.reset()

    def abort(self):
        """Revert ongoing tuning attempt and deactivate algorithm"""
        self.active = False
        if isinstance(self.state_machine.state, Revert):
            # Ongoing decrease.
            self.coeffs[self.coeff_idx] += self.delta_coeffs[self.coeff_idx]
        elif isinstance(self.state_machine.state, Decrease):
            # Ongoing increase.
            self.coeffs[self.coeff_idx] -= self.delta_coeffs[self.coeff_idx]

    def cont(self):
        """Continue tuning"""
        self.active = True
        self.lowest_error = np.inf
        self.state_machine.state = StateMachine.init
        self.coeff_idx = 0


class State(object):
    """Base class for a state in the state state_machine

    Derived classes must implement either also_run() or next().
    """
    def run(self, twiddle):
        """Performs the action of this state"""
        assert(False)

    def also_run(self):
        """Return other state to also be executed during this run of the state machine."""
        return None

    def next(self):
        assert(False)


class Revert(State):
    def run(self, twiddle):
        """ Revert current tuning attempt.coeff_idx

        Also make next attempt with this coefficient less aggressive.
        """
        twiddle.coeffs[twiddle.coeff_idx] += twiddle.delta_coeffs[twiddle.coeff_idx]
        twiddle.delta_coeffs[twiddle.coeff_idx] *= 0.9

    def also_run(self):
        return StateMachine.increase


class Increase(State):
    def run(self, twiddle):
        twiddle.coeff_idx = (twiddle.coeff_idx + 1) % len(twiddle.coeffs)
        if twiddle.coeff_idx == 0:
            twiddle.tuning_count += 1
        twiddle.coeffs[twiddle.coeff_idx] += twiddle.delta_coeffs[twiddle.coeff_idx]

    def next(self):
        return StateMachine.decrease


class Init(State):
    def run(self, twiddle):
        twiddle.coeffs[twiddle.coeff_idx] += twiddle.delta_coeffs[twiddle.coeff_idx]

    def next(self):
        return StateMachine.decrease


class Decrease(State):
    def run(self, twiddle):
        twiddle.coeffs[twiddle.coeff_idx] -= 2.0 * twiddle.delta_coeffs[twiddle.coeff_idx]

    def next(self):
        return StateMachine.revert


class StateMachine(object):
    def __init__(self, initial_state):
        self.state = initial_state

    def run(self, twiddle):
        while True:
            self.state.run(twiddle)
            if self.state.also_run() is None:
                break
            self.state = self.state.also_run()

        self.state = self.state.next()


StateMachine.init = Init()
StateMachine.increase = Increase()
StateMachine.decrease = Decrease()
StateMachine.revert = Revert()

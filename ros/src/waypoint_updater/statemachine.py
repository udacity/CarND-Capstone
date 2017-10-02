import math
import rospy


class FSM(object):
    def __init__(self, states={}, curstate='stopped'):
        self.states = states
        self.currentState = curstate

    def reset(self):
        '''
        Stop the finite state machine
        '''
        self.states = {}

    def addTransition(self, fromState, toState, conditions, callback=None):
        '''
        Add a state transition to the list
        '''
        if fromState not in self.states:
            self.states[fromState] = [[toState, conditions, callback]]
        else:
            self.states[fromState].append([toState, conditions, callback])

    def get_currentState(self):
        return self.currentState
    
    def run(self, conditions=''): 
        assert self.currentState != None
        for tostate in self.states[self.currentState]:
            try:
                #rospy.loginfo(tostate)
                result = tostate[2]()
                if result == True:
                    self.currentState = tostate[0]
                    break # don't check any other state
            except TypeError as e:
                rospy.logwarn(e)
                
    


def dummy_cb(val):
    if val == 10:
        print(val)
        return True
    return False
           
if __name__ == '__main__':
    #fsm = FSM()
    #fsm.addTransition('stopped', 'moving', 'redlight==False', dummy_cb)
    #fsm.addTransition('moving', 'stoped', 'v==0')
    #fsm.run(10)
    #fsm.get_currentState()
    pass


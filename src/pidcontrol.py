import math

class PID_Controller(object):
    '''
    General PID control class. 
    '''

    def __init__(self, Kp = 15.51, Ki = 0.01, Kd = 10.51):
        '''
        Constructs a new PID_Controller object.
        '''
        
        # Parameters
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        
        # State variables
        self.Eprev = 0
        self.Stdt = 0
        self.t = 0
 
    def getCorrection(self, target, actual, dt=0.050):
        '''
        Returns current PID correction based on target value and actual value.
        '''
              
        E = target - actual
    
        dEdt = (E - self.Eprev) / dt if self.t > 0 else 0
        
        self.Stdt += E*dt if self.t > 0 else 0# (E + self.Eprev)*dt if self.t > 0 else 0
   
        correction = self.Kp*E + self.Ki*self.Stdt + self.Kd*dEdt
    
        self.t += 1
        self.Eprev = E
        # if correction > 0 and correction < 10:
        #     return -10.0
        # elif correction < 0 and correction > -10:
        #     return 5.0
        return -correction
    
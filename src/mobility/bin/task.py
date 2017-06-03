import math 

class Task : 
    '''A robot relative place to navigate to. Expressed as r and theta''' 
    def __init__(self, r, theta, delay=0):
        self.r = r 
        self.theta = theta
        self.delay = delay 
             
class TaskState :
    
    TASK_INIT         = 0
    TASK_SEARCH       = 1
    TASK_PICKUP       = 2
    TASK_HOME         = 3
    TASK_DROPOFF      = 4

    def __init__(self):
        self.CurrentState = TaskState.TASK_INIT
        
    def request(self):
        return [Task(0,0,1), Task(1, -math.pi/2)]
        #return None


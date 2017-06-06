#! /usr/bin/env python 

from __future__ import print_function

import sys
import rospy 
import angles
import math
import copy
import StringIO 
import threading
from Queue import Queue 

import tf
from sensor_msgs.msg import Joy
from apriltags_ros.msg import AprilTagDetectionArray 
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Pose2D

from mobility.srv import Command 
from task import Task, TaskState

class Location: 
    '''A class that encodes a handler provided location and accessor methods''' 
    def __init__(self, odo):
        self.Odometry = odo 
    
    def get_pose(self):
        quat = [self.Odometry.pose.pose.orientation.x, 
                self.Odometry.pose.pose.orientation.y, 
                self.Odometry.pose.pose.orientation.z, 
                self.Odometry.pose.pose.orientation.w, 
                ]        
        (r, p, y) = tf.transformations.euler_from_quaternion(quat)
        pose = Pose2D()
        pose.x = self.Odometry.pose.pose.position.x 
        pose.y = self.Odometry.pose.pose.position.y 
        pose.theta = y 
        return pose

    def at_goal(self, goal):
        '''Determine if the pose is within accepable distance of this location''' 
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x, 
                          goal.y - self.Odometry.pose.pose.position.y);
        return (dist < 0.1)
        
class PubSub: 
    '''Keep track of the publishers in this package''' 
    def __init__(self, rover):        
        #rospy.Subscriber(rover + '/joystick', Joy, joystick, queue_size=10)
        rospy.Subscriber(rover + '/mode', UInt8, modeHandler)
        rospy.Subscriber(rover + '/targets', AprilTagDetectionArray, targetHandler)
        rospy.Subscriber(rover + '/obstacle', UInt8, obstacleHandler)
        rospy.Subscriber(rover + '/odom/filtered', Odometry, odometryHandler)
        rospy.Subscriber(rover + '/odom/ekf', Odometry, mapHandler)

        # Services 
        rospy.Service(rover + '/cmd', Command, Debugger)
    
        self.status = rospy.Publisher(rover + '/status', String, queue_size=1, latch=True)
        self.infoLog = rospy.Publisher(rover + '/infoLog', String, queue_size=1, latch=True)
        self.state_machine = rospy.Publisher(rover + '/state_machine', String, queue_size=1, latch=True)
        self.fingerAngle = rospy.Publisher(rover + '/fingerAngle', Float32, queue_size=1, latch=True)
        self.wristAngle = rospy.Publisher(rover + '/wristAngle', Float32, queue_size=1, latch=True)
        self.driveControl = rospy.Publisher(rover + '/driveControl', Twist, queue_size=10)
        self.heartbeat = rospy.Publisher(rover + '/mobility/heartbeat', String, queue_size=1, latch=True)

        self.t = Twist() 
        
    def drive(self, speed, yawerr):
        self.t.linear.x = speed
        self.t.angular.z = yawerr
        self.driveControl.publish(self.t)
        
    def printInfoLog(self, msg):
        s = String()
        s.data = msg 
        self.infoLog.publish(s)

    def printSM(self, msg):
        s = String()
        s.data = msg 
        self.state_machine.publish(s)

    def printStatus(self, msg):
        s = String()
        s.data = msg 
        self.status.publish(s)

class State: 
    '''Global robot state variables''' 
    
    MODE_MANUAL     = 1 
    MODE_AUTO       = 2
    MODE_ALL_AUTO   = 3 

    STATE_INIT      = 0
    STATE_IDLE      = 1
    STATE_TURN      = 2
    STATE_DRIVE     = 3 
    STATE_REVERSE   = 4 
    STATE_PAUSE     = 5 
    STATE_HAZARD    = 6 

    def __init__(self):
        self.Mode = State.MODE_MANUAL
        self.MapLocation = Location(None)
        self.OdomLocation =  Location(None)
        self.CurrentState = State.STATE_INIT
        self.Goal = None
        self.PauseCnt = 0
        self.Controller = TaskState()
        self.Lock = threading.Lock()
        self.Work = Queue()
     
def state_sync(state_func) :
    def wrapper(*args, **kwargs):
        global state
        try:
            state.Lock.acquire()
            return state_func(*args, **kwargs)
        finally:
            state.Lock.release()
    return wrapper

@state_sync
def run(event) :
    global state, pub
        
    if state.Mode == State.MODE_MANUAL :
        pub.drive(0,0)
        return
    
    if state.CurrentState == State.STATE_INIT : 
        state.CurrentState = State.STATE_IDLE
        pub.drive(0,0)

    elif state.CurrentState == State.STATE_IDLE : 
        pub.printSM('WAIT')
        pub.drive(0,0)
        if not state.Work.empty() : 
            task = state.Work.get(False)

            if task.delay > 0 :
                state.PauseCnt = task.delay
                state.CurrentState = State.STATE_PAUSE
            else :                       
                if task.r < 0 :
                    task.theta = 0
    
                cur = state.OdomLocation.get_pose()
                state.Goal = Pose2D()
                state.Goal.theta = cur.theta + task.theta
                state.Goal.x = cur.x + task.r * math.cos(state.Goal.theta)
                state.Goal.y = cur.y + task.r * math.sin(state.Goal.theta)
    
                if task.r < 0 :
                    state.CurrentState = State.STATE_REVERSE
                else:
                    state.CurrentState = State.STATE_TURN

    elif state.CurrentState == State.STATE_TURN :
        pub.printSM('TURN')
        cur = state.OdomLocation.get_pose()
        heading_error = angles.shortest_angular_distance(cur.theta, state.Goal.theta)
        if abs(heading_error) > math.pi / 4 :
            pub.drive(0.05, heading_error)
        else:
            pub.drive(0,0)
            state.CurrentState = State.STATE_DRIVE
            
    elif state.CurrentState == State.STATE_DRIVE :
        pub.printSM('DRIVE')
        cur = state.OdomLocation.get_pose()
        heading_error = angles.shortest_angular_distance(cur.theta, state.Goal.theta)
        goal_angle = angles.shortest_angular_distance(cur.theta, math.atan2(state.Goal.y - cur.y, state.Goal.x - cur.x))
        if state.OdomLocation.at_goal(state.Goal) or abs(goal_angle) > math.pi / 2 :
            pub.drive(0,0)
            state.Goal = None
            state.CurrentState = State.STATE_IDLE            
        elif abs(heading_error) > math.pi / 2 :
            pub.drive(0.05, heading_error)
            state.CurrentState = State.STATE_TURN
        else:
            pub.drive(0.3, heading_error/2)

    elif state.CurrentState == State.STATE_REVERSE :
        pub.printSM('REVERSE')
        cur = state.OdomLocation.get_pose()
        goal_angle = angles.shortest_angular_distance(math.pi + cur.theta, math.atan2(state.Goal.y - cur.y, state.Goal.x - cur.x))
        if state.OdomLocation.at_goal(state.Goal) or abs(goal_angle) > math.pi / 2 : 
            pub.drive(0,0)
            state.Goal = None
            state.CurrentState = State.STATE_IDLE
        else:
            pub.drive(-0.2, 0)

    elif state.CurrentState == State.STATE_PAUSE : 
        pub.printSM('PAUSE')
        pub.drive(0,0)
        if state.PauseCnt == 0 :
            state.CurrentState = State.STATE_IDLE
        else:
            state.PauseCnt = state.PauseCnt - 1
            
def ping(event):
    global pub
    pub.heartbeat.publish("")
    
def joyCmdHandler(msg) :
    pass 

@state_sync
def modeHandler(msg) :
    global state 
    state.Mode = msg

def targetHandler(msg) : 
    pass 

def obstacleHandler(msg) :
    pass 

@state_sync
def odometryHandler(msg) : 
    global state 
    state.OdomLocation = Location(msg)
        
@state_sync
def mapHandler(msg) : 
    global state 
    state.MapLocation = Location(msg)

def Debugger(req):
    redir = StringIO.StringIO()
    save = sys.stdout
    sys.stdout = redir
    err = None
    try :
        exec(req.str)
    except Exception as e :
        err = e

    sys.stdout = save
    if err is not None : 
        return str(err) + "\n"

    rval = redir.getvalue()
    redir.close()
    return str(rval)

def goto(r, theta):
    '''Debugging programmed move.'''
    global state
    t = Task(r, theta)
    state.Work.put(t, False)
    
def stop():
    global state 

def main() : 
    global machine 
    global pub
    global state 
    
    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    rover = sys.argv[1]
    rospy.init_node(rover + '_MOBILITY')

    state = State()
    pub = PubSub(rover)

    rospy.Timer(rospy.Duration(0.1), run)
    rospy.Timer(rospy.Duration(1), ping)
    rospy.spin()

if __name__ == '__main__' : 
    main()

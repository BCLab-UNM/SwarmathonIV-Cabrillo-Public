#! /usr/bin/env python 

import sys
import rospy 
import angles
import math
from sensor_msgs.msg import Joy
from apriltags_ros.msg import AprilTagDetectionArray 
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Pose2D
from twisted.positioning.base import Satellite
import tf
from Queue import Queue 
from mobility.srv import Command 

class Location: 
    '''A class that encodes a handler provided location and accessor methods''' 
    def __init__(self, odo):
        self.Odometry = odo 
    
    def getPose2D(self):
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

    def atGoal(self, goal):
        '''Determine if the pose is within accepable distance of this location''' 
        dist = math.hypot(goal.x - self.Odometry.pose.pose.position.x, 
                          goal.y - self.Odometry.pose.pose.position.y);
        return (dist < 0.1)
        
class Task : 
    '''A robot relative place to navigate to. Expressed as r and theta''' 
    def __init__(self, r, theta):
        self.r = r 
        self.theta = theta
            
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
    
        self.status = rospy.Publisher(rover + '/status', String, queue_size=1)
        self.infoLog = rospy.Publisher(rover + '/infoLog', String, queue_size=1)
        self.state_machine = rospy.Publisher(rover + '/state_machine', String, queue_size=1)
        self.fingerAngle = rospy.Publisher(rover + '/fingerAngle', Float32, queue_size=1)
        self.wristAngle = rospy.Publisher(rover + '/wristAngle', Float32, queue_size=1)
        self.driveControl = rospy.Publisher(rover + '/driveControl', Twist, queue_size=1)
        self.heartbeat = rospy.Publisher(rover + '/mobility/heartbeat', String, queue_size=1)

    def drive(self, speed, yawerr):
        t = Twist()
        t.linear.x = speed
        t.angular.z = yawerr
        self.driveControl.publish(t)
        
class State: 
    '''Global robot state variables''' 
    
    MODE_MANUAL   = 1 
    MODE_AUTO     = 2
    MODE_ALL_AUTO = 3 

    STATE_INIT    = 0
    STATE_WAIT    = 1
    STATE_TURN    = 2
    STATE_DRIVE   = 3 
           
    def __init__(self):
        self.Mode = State.MODE_MANUAL
        self.MapLocation = Location(None)
        self.OdomLocation =  Location(None)
        self.CurrentState = State.STATE_INIT
        self.TaskList = Queue()
        self.Goal = None
        
def run(event) :
    global state, pub
    if state.Mode == State.MODE_MANUAL :
        pub.drive(0,0)
        return
    
    if state.CurrentState == State.STATE_INIT : 
        state.CurrentState = State.STATE_WAIT
        pub.drive(0,0)

    elif state.CurrentState == State.STATE_WAIT : 
        pub.drive(0,0)
        if not state.TaskList.empty() : 
            task = state.TaskList.get(False)
            # Compute the goal.
            cur = state.OdomLocation.getPose2D()
            state.Goal = Pose2D()
            state.Goal.theta = cur.theta + task.theta
            state.Goal.x = cur.x + task.r * math.cos(state.Goal.theta)
            state.Goal.y = cur.y + task.r * math.sin(state.Goal.theta)
            state.CurrentState = State.STATE_TURN
            
    elif state.CurrentState == State.STATE_TURN :
        cur = state.OdomLocation.getPose2D()
        heading_error = angles.shortest_angular_distance(cur.theta, state.Goal.theta)
        if abs(heading_error) > math.pi / 4 :
            pub.drive(0.05, heading_error)
        else:
            pub.drive(0,0)
            state.CurrentState = State.STATE_DRIVE
            
    elif state.CurrentState == State.STATE_DRIVE :
        cur = state.OdomLocation.getPose2D()
        heading_error = angles.shortest_angular_distance(cur.theta, state.Goal.theta)
        goal_angle = angles.shortest_angular_distance(cur.theta, math.atan2(state.Goal.y - cur.y, state.Goal.x - cur.x))
        if not state.OdomLocation.atGoal(state.Goal) :
            if abs(goal_angle) > math.pi / 2 : 
                pub.drive(0.05, heading_error)
                state.CurrentState = State.STATE_TURN
            else:
                pub.drive(0.3, heading_error/2)
        else:
            pub.drive(0,0)
            state.Goal = None
            state.CurrentState = State.STATE_WAIT

def ping(event):
    global pub
    pub.heartbeat.publish("")
    
def joyCmdHandler(msg) :
    pass 

def modeHandler(msg) :
    global state 
    state.Mode = msg

def targetHandler(msg) : 
    pass 

def obstacleHandler(msg) :
    pass 

def odometryHandler(msg) : 
    global state 
    state.OdomLocation = Location(msg)

def mapHandler(msg) : 
    global state 
    state.MapLocation = Location(msg)

def Debugger(req):
    try :
        exec(req.str)
    except Exception as e :
        return str(e)
    return "OK"

def goto(r, theta):
    '''Debugging programmed move.'''
    global state
    t = Task(r, theta)
    state.TaskList.put(t)
    
def stop():
    global state 

def main() : 
    global machine 
    global pub
    global state 
    
    if len(sys.argv) < 2 :
        print 'ussage:', sys.argv[0], '<rovername>'
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

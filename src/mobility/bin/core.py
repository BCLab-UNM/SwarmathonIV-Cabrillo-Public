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
from dynamic_reconfigure.server import Server
from mobility.cfg import DriveConfig 

from mobility.srv import Command, Core
from mobility.msg import MoveResult
from obstacle_detection.msg import Obstacle 

from task import Task, TaskState

def sync(func) :
    '''This decorator forces serial access based on a file level lock. Crude but effective.''' 
    def wrapper(self, *args, **kwargs):
        global StateLock
        try:
            StateLock.acquire()
            return func(self, *args, **kwargs)
        finally:
            StateLock.release()
    return wrapper

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
        return (dist < State.GOAL_DISTANCE_OK)
                    
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
    STATE_STOP      = 5 
    STATE_PAUSE     = 6 
    STATE_HAZARD    = 7 

    DRIVE_MODE_STOP = 0
    DRIVE_MODE_PID  = 1 
    
    # Tunable parameters 
    DRIVE_SPEED_SLOPE = 1 
    DRIVE_SPEED_MIN   = 0.1 
    DRIVE_SPEED_MAX   = 0.5
    
    TURN_SPEED_SLOPE  = 2
    TURN_SPEED_MIN    = 0.1
    TURN_SPEED_MAX    = 0.7
    
    GOAL_DISTANCE_OK  = 0.1

    ROTATE_THRESHOLD  = math.pi / 16 
    DRIVE_ANGLE_ABORT = math.pi / 2 

    STOP_THRESHOLD    = 0.1
               
    def __init__(self, rover):
        self.Mode = State.MODE_MANUAL
        self.MapLocation = Location(None)
        self.OdomLocation =  Location(None)
        self.CurrentState = State.STATE_INIT
        self.Goal = None
        self.Start = None
        self.PauseCnt = 0
        self.Hold = True;
        self.Doing = None
        self.Controller = TaskState()
        self.Work = Queue()
        self.dbg_msg = None
        
        # Subscribers
        #rospy.Subscriber(rover + '/joystick', Joy, joystick, queue_size=10)
        rospy.Subscriber(rover + '/mode', UInt8, self._mode)
        rospy.Subscriber(rover + '/obstacle', Obstacle, self._obstacle)
        rospy.Subscriber(rover + '/odom/filtered', Odometry, self._odom)
        rospy.Subscriber(rover + '/odom/ekf', Odometry, self._map)

        # Services 
        self.control = rospy.Service(rover + '/control', Core, self._control);
        
        # Publishers    
        self.status = rospy.Publisher(rover + '/status', String, queue_size=1, latch=True)
        self.infoLog = rospy.Publisher(rover + '/infoLog', String, queue_size=1, latch=True)
        self.state_machine = rospy.Publisher(rover + '/state_machine', String, queue_size=1, latch=True)
        self.fingerAngle = rospy.Publisher(rover + '/fingerAngle', Float32, queue_size=1, latch=True)
        self.wristAngle = rospy.Publisher(rover + '/wristAngle', Float32, queue_size=1, latch=True)
        self.driveControl = rospy.Publisher(rover + '/driveControl', Twist, queue_size=10)
        self.heartbeat = rospy.Publisher(rover + '/mobility/heartbeat', String, queue_size=1, latch=True)

        # Timers
        rospy.Timer(rospy.Duration(0.1), self.run)
        rospy.Timer(rospy.Duration(1), self._heartbeat)

        # Configuration 
        self.config_srv = Server(DriveConfig, self._reconfigure)

    def _control(self, req):
        print ("Called service.")
        t = Task(req.r, req.theta, req.delay)
        state.Work.put(t, False)
        t.sema.acquire()
        
        rval = MoveResult()
        rval.result = t.result
        return rval
    
    @sync
    def _reconfigure(self, config, level):
        State.DRIVE_SPEED_SLOPE = config['DRIVE_SPEED_SLOPE']
        State.DRIVE_SPEED_MIN   = config['DRIVE_SPEED_MIN']
        State.DRIVE_SPEED_MAX   = config['DRIVE_SPEED_MAX']
        State.TURN_SPEED_SLOPE  = config['TURN_SPEED_SLOPE']
        State.TURN_SPEED_MIN    = config['TURN_SPEED_MIN']
        State.TURN_SPEED_MAX    = config['TURN_SPEED_MAX']
        State.GOAL_DISTANCE_OK  = config['GOAL_DISTANCE_OK']
        State.ROTATE_THRESHOLD  = config['ROTATE_THRESHOLD']
        State.DRIVE_ANGLE_ABORT = config['DRIVE_ANGLE_ABORT']
        self.print_info_log('Mobility parameter reconfiguration done.')
        return config 
    
    def _heartbeat(self, event):
        self.heartbeat.publish("ok")
        
    @sync
    def _joystick(self, msg) :
        pass 

    @sync    
    def _mode(self, msg) :
        self.Mode = msg
    
    @sync
    def _obstacle(self, msg) :
        if self.Mode == State.MODE_MANUAL or self.CurrentState != State.STATE_DRIVE :
            return
        
        if (msg.msg & Obstacle.IS_SONAR) != 0 :
            self.Doing.result = MoveResult.OBSTACLE_SONAR 

        if (msg.msg & Obstacle.IS_VISION) != 0 :
            self.Doing.result = MoveResult.OBSTACLE_VISION

        self.CurrentState = State.STATE_STOP
        while not self.Work.empty() :
            self.work.get(False)
        
    @sync    
    def _odom(self, msg) : 
        self.OdomLocation = Location(msg)
            
    @sync    
    def _map(self, msg) : 
        self.MapLocation = Location(msg)

    def drive(self, linear, angular, mode):
        t = Twist() 
        t.linear.x = linear
        t.angular.y = mode
        t.angular.z = angular
        self.driveControl.publish(t)
        
    def print_info_log(self, msg):
        s = String()
        s.data = msg 
        self.infoLog.publish(s)

    def print_debug(self, msg):
        if self.dbg_msg is None or self.dbg_msg != msg: 
            s = String()
            s.data = msg 
            self.state_machine.publish(s)
        self.dbg_msg = msg 
        
    def print_status(self, msg):
        s = String()
        s.data = msg 
        self.status.publish(s)

    @sync    
    def run(self, event) :            
        if self.Mode == State.MODE_MANUAL :
            self.print_debug('Manual Mode')
            self.drive(0,0,State.DRIVE_MODE_STOP)
            return
        
        if self.CurrentState == State.STATE_INIT : 
            self.print_debug('INIT')
            self.CurrentState = State.STATE_IDLE
            self.drive(0,0,State.DRIVE_MODE_STOP)
    
        elif self.CurrentState == State.STATE_IDLE : 
            self.print_debug('IDLE')
            self.drive(0,0,State.DRIVE_MODE_STOP)

            if self.Doing is not None : 
                # FIXME: Place result here
                self.Doing.sema.release()
                self.Doing = None
                
            if not self.Work.empty() : 
                self.Doing = self.Work.get(False)
    
                if self.Doing.delay > 0 :
                    self.PauseCnt = self.Doing.delay
                    self.CurrentState = State.STATE_PAUSE
                else :                       
                    if self.Doing.r < 0 :
                        self.Doing.theta = 0
        
                    cur = self.OdomLocation.get_pose()
                    self.Goal = Pose2D()
                    self.Goal.theta = cur.theta + self.Doing.theta
                    self.Goal.x = cur.x + self.Doing.r * math.cos(self.Goal.theta)
                    self.Goal.y = cur.y + self.Doing.r * math.sin(self.Goal.theta)
                    self.Start = cur
                    self.Hold = self.Doing.hold
                    
                    if self.Doing.r < 0 :
                        self.CurrentState = State.STATE_REVERSE
                    else:
                        self.CurrentState = State.STATE_TURN
    
        elif self.CurrentState == State.STATE_TURN :
            self.print_debug('TURN')
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            if abs(heading_error) > State.ROTATE_THRESHOLD :
                self.drive(0, get_turn(self.Start, self.Goal, cur), State.DRIVE_MODE_PID)
            else:
                self.CurrentState = State.STATE_DRIVE
                
        elif self.CurrentState == State.STATE_DRIVE :
            self.print_debug('DRIVE')
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            goal_angle = angles.shortest_angular_distance(cur.theta, math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal) or abs(goal_angle) > State.DRIVE_ANGLE_ABORT :
                self.Goal = None
                if self.Hold :
                    self.CurrentState = State.STATE_STOP
                else:
                    self.CurrentState = State.STATE_IDLE
                    
            elif abs(heading_error) > math.pi / 2 :
                self.Doing.result = MoveResult.PATH_FAIL
                self.CurrentState = State.STATE_STOP
            else:
                self.drive(get_speed(self.Start, self.Goal, cur), heading_error/2, State.DRIVE_MODE_PID)
    
        elif self.CurrentState == State.STATE_REVERSE :
            self.print_debug('REVERSE')
            cur = self.OdomLocation.get_pose()
            goal_angle = angles.shortest_angular_distance(math.pi + cur.theta, math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal) or abs(goal_angle) > State.DRIVE_ANGLE_ABORT : 
                self.Goal = None
                if self.Hold :
                    self.CurrentState = State.STATE_STOP
                else:
                    self.CurrentState = State.STATE_IDLE
            else:
                self.drive(-0.2, 0, State.DRIVE_MODE_PID)
    
        elif self.CurrentState == State.STATE_PAUSE : 
            self.print_debug('PAUSE')
            self.drive(0, 0, State.DRIVE_MODE_STOP)
            if self.PauseCnt == 0 :
                self.CurrentState = State.STATE_IDLE
            else:
                self.PauseCnt = self.PauseCnt - 1

        elif self.CurrentState == State.STATE_STOP : 
            self.print_debug('STOP')
            self.drive(0, 0, State.DRIVE_MODE_STOP)
            cur = self.OdomLocation.Odometry
            if cur.twist.twist.angular.z < State.STOP_THRESHOLD and cur.twist.twist.linear.x < State.STOP_THRESHOLD :
                self.CurrentState = State.STATE_IDLE
            
def get_turn(start, end, current):
    dist_from_start = angles.shortest_angular_distance(start.theta, current.theta)
    dist_to_end = angles.shortest_angular_distance(current.theta, end.theta) - State.ROTATE_THRESHOLD
    
    def dist_to_turn(dist):
        speed = dist * State.TURN_SPEED_SLOPE
        if speed > 0 : 
            if speed > State.TURN_SPEED_MAX : 
                speed = State.TURN_SPEED_MAX
            elif speed < State.TURN_SPEED_MIN : 
                speed = State.TURN_SPEED_MIN
        else :
            if speed < -State.TURN_SPEED_MAX :
                speed = -State.TURN_SPEED_MAX
            elif speed > -State.TURN_SPEED_MIN :
                speed = -State.TURN_SPEED_MIN
                         
        return speed

    to = dist_to_turn(dist_to_end)
    if to > 0 :
        return -min(to, -abs(dist_to_turn(dist_from_start)))
    else:
        return -max(to, abs(dist_to_turn(dist_from_start)))

def get_speed(start, end, current):
    dist_from_start = abs(math.hypot(start.x - current.x, start.y - current.y))
    dist_to_end = abs(math.hypot(current.x - end.x, current.y - end.y)) - State.GOAL_DISTANCE_OK
    
    def dist_to_speed(dist):
        speed = dist * State.DRIVE_SPEED_SLOPE
        if speed > State.DRIVE_SPEED_MAX:
            speed = State.DRIVE_SPEED_MAX
        elif speed < State.DRIVE_SPEED_MIN :
            speed = State.DRIVE_SPEED_MIN 
        
        return speed

    return min(dist_to_speed(dist_from_start), dist_to_speed(dist_to_end))    
     
def debug(req):
    global state
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

def stop():
    global state
    state.Mode = State.MODE_MANUAL
    state.CurrentState = State.STATE_IDLE
    while not self.Work.empty() :
        self.work.get(False)

def go():
    global state
    state.Mode = State.MODE_AUTO

def goto(r, theta):
    '''Debugging programmed move.'''
    global state
    t = Task(r, theta)
    state.Work.put(t, False)
        
def main() :     
    global StateLock 
    global state

    if len(sys.argv) < 2 :
        print('usage:', sys.argv[0], '<rovername>')
        exit (-1)
    
    StateLock = threading.Lock()
    rover = sys.argv[1]
    rospy.init_node(rover + '_MOBILITY')
    state = State(rover)
    rospy.Service(rover + '/cmd', Command, debug)
    rospy.spin()

if __name__ == '__main__' : 
    main()

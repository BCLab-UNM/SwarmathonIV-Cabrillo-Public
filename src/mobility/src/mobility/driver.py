#! /usr/bin/env python 

from __future__ import print_function

import sys
import rospy 
import angles
import math
import copy
import thread
import threading

from Queue import Queue 

import tf
from sensor_msgs.msg import Joy
from std_msgs.msg import UInt8, String, Float32
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Twist, Pose2D
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client

from mobility.cfg import driveConfig 
from mobility.srv import Core
from mobility.msg import MoveResult
from swarmie_msgs.msg import Obstacle
from angles import shortest_angular_distance

import threading 
package_lock = threading.Lock()

from mobility import sync
from mobility.swarmie import Location

class Task : 
    '''A robot relative place to navigate to. Expressed as r and theta''' 
    
    def __init__(self, msg, blocking=True):
        self.request = msg
        self.result = MoveResult.SUCCESS
        if blocking :
            self.sema = threading.Semaphore(0)
        else:
            self.sema = None 
            
class State: 
    '''Global robot state variables''' 

    STATE_IDLE      = 0
    STATE_TURN      = 1
    STATE_DRIVE     = 2 
    STATE_REVERSE   = 3 
    STATE_TIMED     = 4 

    DRIVE_MODE_STOP = 0
    DRIVE_MODE_PID  = 1 
    
    # Tune these with dynaimc reconfigure.
    DRIVE_SPEED                = 0
    REVERSE_SPEED              = 0 
    TURN_SPEED                 = 0
    HEADING_RESTORE_FACTOR     = 0
    GOAL_DISTANCE_OK           = 0
    ROTATE_THRESHOLD           = 0
    DRIVE_ANGLE_ABORT          = 0

    DRIVE_SPEED_MAX            = 0.6
    TURN_SPEED_MAX             = 1.2 
    
    def __init__(self):
        self.MapLocation = Location(None)
        self.OdomLocation =  Location(None)
        self.CurrentState = State.STATE_IDLE
        self.Goal = None
        self.Start = None
        self.TimerCount = 0
        self.Doing = None
        self.Work = Queue()
        self.dbg_msg = None
        self.Obstacles = 0 
        self.JoystickCommand = Joy() 
        self.JoystickCommand.axes = [0,0,0,0,0,0]
        
        # Configuration 
        State.DRIVE_SPEED = rospy.get_param("DRIVE_SPEED", default=0.3)
        State.REVERSE_SPEED = rospy.get_param("REVERSE_SPEED", default=0.2)
        State.TURN_SPEED = rospy.get_param("TURN_SPEED", default=0.6)
        State.HEADING_RESTORE_FACTOR = rospy.get_param("HEADING_RESTORE_FACTOR", default=2)
        State.GOAL_DISTANCE_OK = rospy.get_param("GOAL_DISTANCE_OK", default=0.1)
        State.ROTATE_THRESHOLD = rospy.get_param("ROTATE_THRESHOLD", default=math.pi/16)
        State.DRIVE_ANGLE_ABORT = rospy.get_param("DRIVE_ANGLE_ABORT", default=math.pi/4)

        # Subscribers
        rospy.Subscriber('joystick', Joy, self._joystick, queue_size=10)
        rospy.Subscriber('obstacle', Obstacle, self._obstacle)
        rospy.Subscriber('odom/filtered', Odometry, self._odom)

        # Services 
        self.control = rospy.Service('control', Core, self._control);
        
        # Publishers
        self.state_machine = rospy.Publisher('state_machine', String, queue_size=1, latch=True)
        self.driveControl = rospy.Publisher('driveControl', Twist, queue_size=10)

        # Configuration 
        self.config_srv = Server(driveConfig, self._reconfigure)

        # Start a thread to do initial configuration.
        thread.start_new_thread(self.do_initial_config, ())
        
    def _stop_now(self, result) :
        self.CurrentState = State.STATE_IDLE
        while not self.Work.empty() :
            item = self.Work.get(False)
            item.result = result
            if item.sema is not None :
                item.sema.release()

        if self.Doing is not None :
            self.Doing.result = result
    
    def _control(self, req):
        for r in req.req[:-1] :
            self.Work.put(Task(r, False), False)
        
        r = req.req[-1]
        t = Task(r, True)
        self.Work.put(t, True)

        sleep_wait = 0.2
        sleep_turns = r.timeout / sleep_wait
        while not t.sema.acquire(blocking=False) :
            rospy.sleep(sleep_wait)
            sleep_turns -= 1
            if sleep_turns == 0 :
                # Ugh. Is this safe?
                with package_lock :
                    self._stop_now(MoveResult.TIMEOUT)

        rval = MoveResult()
        rval.result = t.result
        return rval
    
    @sync(package_lock)
    def _reconfigure(self, config, level):
        State.DRIVE_SPEED = config["DRIVE_SPEED"]
        State.REVERSE_SPEED = config["REVERSE_SPEED"]
        State.TURN_SPEED = config["TURN_SPEED"]
        State.HEADING_RESTORE_FACTOR = config["HEADING_RESTORE_FACTOR"]
        State.GOAL_DISTANCE_OK = config["GOAL_DISTANCE_OK"]
        State.ROTATE_THRESHOLD = config["ROTATE_THRESHOLD"]
        State.DRIVE_ANGLE_ABORT = config["DRIVE_ANGLE_ABORT"]
        self.print_debug('Mobility parameter reconfiguration done.')
        return config 

    @sync(package_lock)
    def _joystick(self, joy_command):
        self.JoystickCommand = joy_command 
    
    @sync(package_lock)
    def set_mode(self, msg) :
        if msg.data == 1 :
            self._stop_now(MoveResult.USER_ABORT)
    
    def __check_obstacles(self):
        if self.Doing is not None :        
            detected = self.Obstacles & self.Doing.request.obstacles
         
            if (detected & Obstacle.IS_SONAR) != 0 :
                self._stop_now(MoveResult.OBSTACLE_SONAR)

            if (detected & Obstacle.IS_VISION) != 0 :
                if detected & Obstacle.INSIDE_HOME:
                    self._stop_now(MoveResult.INSIDE_HOME)
                elif detected & Obstacle.TAG_HOME:
                    self._stop_now(MoveResult.OBSTACLE_HOME)
                elif detected & Obstacle.HOME_CORNER:
                    self._stop_now(MoveResult.OBSTACLE_CORNER)
                else:
                    self._stop_now(MoveResult.OBSTACLE_TAG)

    @sync(package_lock)
    def _obstacle(self, msg) :
        self.Obstacles &= ~msg.mask 
        self.Obstacles |= msg.msg 
        self.__check_obstacles()        
        
    @sync(package_lock)
    def _odom(self, msg) : 
        self.OdomLocation.Odometry = msg

    def drive(self, linear, angular, mode):
        t = Twist() 
        t.linear.x = linear
        t.angular.y = mode
        t.angular.z = angular
        self.driveControl.publish(t)
        
    def print_debug(self, msg):
        if self.dbg_msg is None or self.dbg_msg != msg: 
            s = String()
            s.data = msg 
            self.state_machine.publish(s)
        self.dbg_msg = msg 
        
    @sync(package_lock)
    def run(self) :            
        if self.CurrentState == State.STATE_IDLE : 
            self.print_debug('IDLE')

            if self.Doing is not None : 
                if self.Doing.sema is not None :
                    self.Doing.sema.release()
                self.Doing = None
                
            if self.Work.empty() : 
                # Let the joystick drive.
                lin = self.JoystickCommand.axes[4] * State.DRIVE_SPEED
                ang = self.JoystickCommand.axes[3] * State.TURN_SPEED
                if abs(lin) < 0.1 and abs(ang) < 0.1 :
                    self.drive(0, 0, State.DRIVE_MODE_STOP)
                else:
                    self.drive(lin, ang, State.DRIVE_MODE_PID)
            else:
                self.Doing = self.Work.get(False)
    
                if self.Doing.request.timer > 0 :
                    self.TimerCount = self.Doing.request.timer * 10
                    self.CurrentState = State.STATE_TIMED
                else :                       
                    if self.Doing.request.r < 0 :
                        self.Doing.request.theta = 0
        
                    req_theta = self.Doing.request.theta 
                    if req_theta > 0 : 
                        req_theta += State.ROTATE_THRESHOLD / 2.0 
                    elif req_theta < 0 :
                        req_theta -= State.ROTATE_THRESHOLD / 2.0

                    req_r = self.Doing.request.r 
                    if req_r > 0 : 
                        req_r += State.GOAL_DISTANCE_OK / 2.0
                    elif req_r < 0 :
                        req_r -= State.GOAL_DISTANCE_OK / 2.0
                    
                    if self.Doing.request.linear > State.DRIVE_SPEED_MAX:
                        self.Doing.request.linear = State.DRIVE_SPEED_MAX
                    elif self.Doing.request.linear <= 0:
                        self.Doing.request.linear = State.DRIVE_SPEED
                        
                    if self.Doing.request.angular > State.TURN_SPEED_MAX:
                        self.Doing.request.angular = State.TURN_SPEED_MAX
                    elif self.Doing.request.angular <= 0:
                        self.Doing.request.angular = State.TURN_SPEED
                                            
                    cur = self.OdomLocation.get_pose()
                    self.Goal = Pose2D()
                    self.Goal.theta = cur.theta + req_theta
                    self.Goal.x = cur.x + req_r * math.cos(self.Goal.theta)
                    self.Goal.y = cur.y + req_r * math.sin(self.Goal.theta)
                    self.Start = cur
                    
                    if self.Doing.request.r < 0 :
                        self.CurrentState = State.STATE_REVERSE
                    else:
                        self.CurrentState = State.STATE_TURN
                        
                self.__check_obstacles()

        elif self.CurrentState == State.STATE_TURN :
            self.print_debug('TURN')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            if abs(heading_error) > State.ROTATE_THRESHOLD :
                if heading_error < 0 :
                    self.drive(0, -self.Doing.request.angular, State.DRIVE_MODE_PID)
                else:
                    self.drive(0, self.Doing.request.angular, State.DRIVE_MODE_PID)
            else:
                self.CurrentState = State.STATE_DRIVE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
                
        elif self.CurrentState == State.STATE_DRIVE :
            self.print_debug('DRIVE')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            goal_angle = angles.shortest_angular_distance(cur.theta, math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal, State.GOAL_DISTANCE_OK) or abs(goal_angle) > State.DRIVE_ANGLE_ABORT :
                self.Goal = None
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
                    
            elif abs(heading_error) > State.DRIVE_ANGLE_ABORT / 2 :
                self._stop_now(MoveResult.PATH_FAIL)
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(self.Doing.request.linear,
                           heading_error * State.HEADING_RESTORE_FACTOR,
                           State.DRIVE_MODE_PID)
    
        elif self.CurrentState == State.STATE_REVERSE :
            self.print_debug('REVERSE')
            self.__check_obstacles()
            cur = self.OdomLocation.get_pose()
            heading_error = angles.shortest_angular_distance(cur.theta, self.Goal.theta)
            goal_angle = angles.shortest_angular_distance(math.pi + cur.theta, math.atan2(self.Goal.y - cur.y, self.Goal.x - cur.x))
            if self.OdomLocation.at_goal(self.Goal, State.GOAL_DISTANCE_OK) or abs(goal_angle) > State.DRIVE_ANGLE_ABORT : 
                self.Goal = None
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            elif abs(heading_error) > State.DRIVE_ANGLE_ABORT / 2 :
                self._stop_now(MoveResult.PATH_FAIL)
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(-State.REVERSE_SPEED, 
                           heading_error * State.HEADING_RESTORE_FACTOR, 
                           State.DRIVE_MODE_PID)
    
        elif self.CurrentState == State.STATE_TIMED : 
            self.print_debug('TIMED')
            self.__check_obstacles()
            if self.Doing.request.linear == 0 and self.Doing.request.angular == 0 :
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.drive(self.Doing.request.linear, self.Doing.request.angular, State.DRIVE_MODE_PID)
                
            if self.TimerCount == 0 :
                self.CurrentState = State.STATE_IDLE
                self.drive(0, 0, State.DRIVE_MODE_STOP)
            else:
                self.TimerCount = self.TimerCount - 1

    def do_initial_config(self):
        # Do initial configuration. 
        params = {
            "DRIVE_SPEED": State.DRIVE_SPEED,
            "REVERSE_SPEED": State.REVERSE_SPEED,
            "TURN_SPEED": State.TURN_SPEED,
            "HEADING_RESTORE_FACTOR": State.HEADING_RESTORE_FACTOR,
            "GOAL_DISTANCE_OK": State.GOAL_DISTANCE_OK,
            "ROTATE_THRESHOLD": State.ROTATE_THRESHOLD,
            "DRIVE_ANGLE_ABORT": State.DRIVE_ANGLE_ABORT,
            }
        dyn_client = Client('mobility') 
        dyn_client.update_configuration(params)
        print ('Initial configuration sent.')

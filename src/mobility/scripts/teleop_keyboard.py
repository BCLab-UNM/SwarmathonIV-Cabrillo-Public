#!/usr/bin/env python
"""
Modified from ros-teleop/teleop_twist_keyboard.
https://github.com/ros-teleop/teleop_twist_keyboard
Uses Swarmie API to drive the rover with keyboard.
"""
# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import math
import sys
import select
import termios
import textwrap
import tty
import rospy

# from geometry_msgs.msg import Twist
from swarmie_msgs.msg import Obstacle
from mobility.swarmie import Swarmie

msg = """
Reading from the keyboard and driving using Swarmie API!
-----------------    -------------------
Moving around:       Fingers (use the shift key):
   u    i    o       U         O
   j    k    l       (close)   (open)
   m    ,    .                  

Wrist:
-----------------
t : up
g : middle
b : down

anything else : stop

q/z : increase/decrease linear speed by 10%
Q/Z : incread/decrease angular speed by 10%
w/x : increase/decrease drive distance by 10%
e/c : increase/decrease turn angle by 10%

Toggle Obstacles to ignore:
-----------------
Press * for the menu (not implemented)

CTRL-C to quit
"""

obstacle_msg = """
Toggle Obstacles to ignore:
-----------------
PATH_IS_CLEAR    = 0
SONAR_LEFT       = 1
SONAR_RIGHT      = 2
SONAR_CENTER     = 4
SONAR_BLOCK      = 8
TAG_TARGET       = 256
TAG_HOME         = 512

IS_SONAR         = 15
IS_VISION        = 768
"""


def get_key():
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def params_msg(drive_speed, turn_speed, drive_dist, turn_theta):
    msg = '''currently:\tdrive speed {}\tturn speed {}
    drive dist {} turn theta {}'''
    return textwrap.dedent(msg.format(
        drive_speed,
        turn_speed,
        drive_dist,
        turn_theta
    ))


def main():

    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername, anonymous=True)

    global settings
    settings = termios.tcgetattr(sys.stdin)

    # pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    # rospy.init_node('teleop_keyboard')

    drive_bindings = {
        'i': 1,  # positive
        # 'o':(1,0,0,-1),
        # 'u':(1,0,0,1),
        ',': -1,  # negative
        # '.':(-1,0,0,1),
        # 'm':(-1,0,0,-1),
        # 'I':(1,0,0,0),
        # 'J':(0,1,0,0),
        # 'L':(0,-1,0,0),
        # '<':(-1,0,0,0),
        # '>':(-1,-1,0,0),
        # 'M':(-1,1,0,0),
    }

    turn_bindings = {
        'j': 1,  # turn left, positive theta
        'l': -1,  # turn right, negative theta
    }

    claw_bindings = {
        'O': swarmie.fingers_open,  # fingers open
        'U': swarmie.fingers_close,  # fingers close
        't': swarmie.wrist_up,  # wrist up
        'g': swarmie.wrist_middle,  # wrist middle
        'b': swarmie.wrist_down,  # wrist down
    }

    param_bindings = {
        'q': ['drive_speed', 1.1],
        'z': ['drive_speed', 0.9],
        'Q': ['turn_speed', 1.1],
        'Z': ['turn_speed', 0.9],
        'w': ['drive_dist', 1.1],
        'x': ['drive_dist', 0.9],
        'e': ['turn_theta', 1.1],
        'c': ['turn_theta', 0.9],
    }

    params = {
        'drive_speed': rospy.get_param(
            rovername + '_MOBILITY/DRIVE_SPEED',
            0.2
        ),
        'turn_speed': rospy.get_param(
            rovername + '_MOBILITY/TURN_SPEED',
            0.6
        ),
        'drive_dist': 0.5,
        'turn_theta': math.pi/2
    }
    ignore_obstacles = Obstacle.IS_SONAR | Obstacle.IS_VISION

    status = 0

    try:
        print msg
        print params_msg(
            params['drive_speed'],
            params['turn_speed'],
            params['drive_dist'],
            params['turn_theta']
        )
        while True:
            key = get_key()
            if key in drive_bindings.keys():
                dist = params['drive_dist']
                if drive_bindings[key] < 0:
                    dist = -dist
                swarmie.drive(dist, ignore=ignore_obstacles)
            elif key in turn_bindings.keys():
                theta = params['turn_theta']
                if turn_bindings[key] < 0:
                    theta = -theta
                swarmie.turn(theta, ignore=ignore_obstacles)
            elif key in claw_bindings.keys():
                claw_bindings[key]()  # call the function at that key
            elif key in param_bindings.keys():
                params[param_bindings[key][0]] *= param_bindings[key][1]
                if param_bindings[key] == 'drive_speed':
                    rospy.set_param(
                        rovername + '_MOBILITY/DRIVE_SPEED',
                        param_bindings[key][1]
                    )
                elif param_bindings[key] == 'turn_speed':
                    rospy.set_param(
                        rovername + '_MOBILITY/TURN_SPEED',
                        param_bindings[key][1]
                    )

                print params_msg(
                    params['drive_speed'],
                    params['turn_speed'],
                    params['drive_dist'],
                    params['turn_theta']
                )
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            else:
                swarmie.stop()
                if (key == '\x03'):
                    break

            swarmie.stop()
            # twist = Twist()
            # twist.linear.x = x*speed
            # twist.linear.y = y*speed
            # twist.linear.z = z*speed
            # twist.angular.x = 0
            # twist.angular.y = 0
            # twist.angular.z = th*turn
            # pub.publish(twist)

    except Exception as e:
        print e

    finally:
        swarmie.stop()
        # twist = Twist()
        # twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        # twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        # pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()

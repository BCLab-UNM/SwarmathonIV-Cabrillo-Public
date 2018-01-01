#!/usr/bin/env python
"""
Modified from ros-teleop/teleop_twist_keyboard.
https://github.com/ros-teleop/teleop_twist_keyboard
Uses Swarmie API to drive the rover with keyboard.

Questions/comments:
- Would be nice to be able to call stop_now() or something to stop
the rover immediately. Since right now, all obstacles are ignored
while driving.
"""
import math
import sys
import select
import termios
import textwrap
import tty
import rospy

import dynamic_reconfigure.client
from swarmie_msgs.msg import Obstacle
from mobility.swarmie import Swarmie

msg = """
Reading from the keyboard and driving using Swarmie API!
-----------------    -------------------
Moving around:       Fingers (use the shift key):
        i            U         O
   j    k    l       (close)   (open)
        ,                       

Wrist:
-----------------
t : up
g : middle
b : down

anything else : stop

q/z : increase/decrease drive speed by 10%
Q/Z : increase/decrease reverse speed by 10%
w/x : incread/decrease turn speed by 10%
e/c : increase/decrease drive distance by 10%
r/v : increase/decrease turn angle by 10%

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


def params_msg(drive_speed, reverse_speed, turn_speed, drive_dist, turn_theta):
    msg = '''currently:
    drive speed (q/z): {}
    reverse speed (Q/Z): {}
    turn speed (w/x): {}
    drive dist (e/c): {}
    turn theta (r/v): {}'''
    return textwrap.dedent(msg.format(
        drive_speed,
        reverse_speed,
        turn_speed,
        drive_dist,
        turn_theta
    ))


def update_params(config):
    global params
    params['drive_speed'] = config['DRIVE_SPEED']
    params['reverse_speed'] = config['REVERSE_SPEED']
    params['turn_speed'] = config['TURN_SPEED']


def main():

    if len(sys.argv) < 2:
        print('usage:', sys.argv[0], '<rovername>')
        exit(-1)

    rovername = sys.argv[1]
    swarmie = Swarmie(rovername, anonymous=True)

    global settings
    global params
    params = {}

    settings = termios.tcgetattr(sys.stdin)
    params['drive_dist'] = 0.5
    params['turn_theta'] = math.pi / 2


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
        'Q': ['reverse_speed', 1.1],
        'Z': ['reverse_speed', 0.9],
        'w': ['turn_speed', 1.1],
        'x': ['turn_speed', 0.9],
        'e': ['drive_dist', 1.1],
        'c': ['drive_dist', 0.9],
        'r': ['turn_theta', 1.1],
        'v': ['turn_theta', 0.9],
    }

    param_client = dynamic_reconfigure.client.Client(
        rovername + '_MOBILITY',
        config_callback=update_params
    )
    server_config = param_client.get_configuration()
    ignore_obstacles = Obstacle.IS_SONAR | Obstacle.IS_VISION

    status = 0

    try:
        print msg
        print params_msg(
            params['drive_speed'],
            params['reverse_speed'],
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
                if param_bindings[key][0] == 'drive_speed':
                    param_client.update_configuration(
                        {'DRIVE_SPEED': params['drive_speed']}
                    )
                elif param_bindings[key][0] == 'reverse_speed':
                    param_client.update_configuration(
                        {'REVERSE_SPEED': params['reverse_speed']}
                    )
                elif param_bindings[key][0] == 'turn_speed':
                    param_client.update_configuration(
                        {'TURN_SPEED': params['turn_speed']}
                    )
                # Update params once now to make sure no params were
                # set to invalid values.
                server_config = param_client.get_configuration()
                update_params(server_config)

                print params_msg(
                    params['drive_speed'],
                    params['reverse_speed'],
                    params['turn_speed'],
                    params['drive_dist'],
                    params['turn_theta']
                )
                if (status == 9):
                    print msg
                status = (status + 1) % 10
            else:
                swarmie.stop()
                if (key == '\x03'):
                    break

            swarmie.stop()

    except Exception as e:
        print e

    finally:
        swarmie.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()

#! /usr/bin/env python
"""
Modified from ros-teleop/teleop_twist_keyboard.
https://github.com/ros-teleop/teleop_twist_keyboard
Uses Swarmie API to drive the rover with keyboard.
"""
from __future__ import print_function
import math
import sys
import select
import termios
import textwrap
import traceback
import tty
import rospy
import rosnode

import dynamic_reconfigure.client
from swarmie_msgs.msg import Obstacle
from mobility.swarmie import Swarmie, TagException, HomeException, ObstacleException, PathException, AbortException

msg = """
Reading from the keyboard and driving using Swarmie API!
--------------------------------------------------------
CTRL-C to quit
-----------------    -------------------
Moving around:       Fingers (use the shift key):
        i            U         O
   j    k    l       (close)   (open)
        ,                       

Wrist:               Driving Parameters:
-----------------    -----------------------------------
t : up               1/2 : -/+ drive speed by 10%
g : middle           3/4 : -/+ reverse speed by 10%
b : down             5/6 : -/+ turn speed by 10%
                     I/< : -/+ drive distance by 10%
                     J/L : -/+ turn angle by 10%

anything else : stop (not implemented)
--------------------------------------------------------"""



def get_key():
    global settings
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def obstacle_msg(ignore_obstacles):
    obstacles = [
        ['PATH_IS_CLEAR', 0],
        ['SONAR_LEFT', 1],
        ['SONAR_RIGHT', 2],
        ['SONAR_CENTER', 4],
        ['SONAR_BLOCK', 8],
        ['TAG_TARGET', 256],
        ['TAG_HOME', 512],
        ['IS_SONAR', 15],
        ['IS_VISION', 768]
    ]
    msg = """
    Toggle Obstacles to ignore (* = currently ignored):
    -----------------
    (!) {:<14}= 0{}
    (a) {:<14}= 1{}
    (d) {:<14}= 2{}
    (s) {:<14}= 4{}
    (f) {:<14}= 8{}
    (T) {:<14}= 256{}
    (H) {:<14}= 512{}

    (S) {:<14}= 15{}
    (V) {:<14}= 768{}
    """
    msg_formatter = []
    if ignore_obstacles == 0:
        msg_formatter = ['*', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ']
    else:
        msg_formatter.append('')
        for i in xrange(1, len(obstacles)):
            if obstacles[i][1] & ignore_obstacles == obstacles[i][1]:
                msg_formatter.append('*')
            else:
                msg_formatter.append(' ')
    result = []
    for obstacle, formatter in zip(obstacles, msg_formatter):
        result.append(obstacle[0])
        result.append(formatter)
    return textwrap.dedent(msg.format(*result))



def params_msg(drive_speed, reverse_speed, turn_speed, drive_dist, turn_theta):
    msg = '''Currently:
    {:<20}: {:.2f} (m/s)
    {:<20}: {:.2f} (m/s)
    {:<20}: {:.2f} (rad/s)
    {:<20}: {:.2f} (m)
    {:<20}: {:.2f} (rad)'''
    return textwrap.dedent(msg.format(
        'drive speed (1/2)',
        drive_speed,
        'reverse speed (3/4)',
        reverse_speed,
        'turn speed (5/6)',
        turn_speed,
        'drive dist (I/<)',
        drive_dist,
        'turn theta (J/L)',
        turn_theta
    ))


def update_params(config):
    global params
    params['drive_speed'] = config['DRIVE_SPEED']
    params['reverse_speed'] = config['REVERSE_SPEED']
    params['turn_speed'] = config['TURN_SPEED']


def main():

    if len(sys.argv) < 2:
        rovers = set()
        nodes = rosnode.get_node_names()
        for node in nodes:
            if 'MOBILITY' in node:
                node = node.lstrip('/')
                rovername = node.split('_')[0]
                rovers.add(rovername)
        if len(rovers) == 0:
            print('\033[91m',"No Rovers Detected",'\033[0m')
            print('usage:', sys.argv[0], '<rovername>')
            exit(-1)
        elif len(rovers) == 1:
            rovername = rovers.pop()
            print('Detected rovers: ', rovername)
            print('\033[92m',"Auto selected:",rovername,'\033[0m')
        else:
            print('Detected rovers:')
            for rover in rovers:
                print(rover)
            rovername = ''
            while rovername not in rovers:
                rovername = raw_input('Which rover would you like to connect to? ')
    else:
        rovername = sys.argv[1]

    swarmie = Swarmie(rovername, node_suffix='_teleop_keyboard')

    global settings
    settings = termios.tcgetattr(sys.stdin)

    global params
    params = {}
    params['drive_dist'] = 0.5
    params['turn_theta'] = math.pi / 2

    drive_bindings = {
        'i': 1,  # positive
        ',': -1,  # negative
    }
    turn_bindings = {
        'j': 1,  # turn left, positive theta
        'l': -1,  # turn right, negative theta
    }
    obstacle_bindings = {
        '!': Obstacle.PATH_IS_CLEAR,
        'a': Obstacle.SONAR_LEFT,
        's': Obstacle.SONAR_CENTER,
        'd': Obstacle.SONAR_RIGHT,
        'f': Obstacle.SONAR_BLOCK,
        'T': Obstacle.TAG_TARGET,
        'H': Obstacle.TAG_HOME,
        'S': Obstacle.IS_SONAR,
        'V': Obstacle.IS_VISION
    }
    claw_bindings = {
        'O': swarmie.fingers_open,
        'U': swarmie.fingers_close,
        't': swarmie.wrist_up,
        'g': swarmie.wrist_middle,
        'b': swarmie.wrist_down,
    }
    param_bindings = {
        '2': ['drive_speed', 1.1],
        '1': ['drive_speed', 0.9],
        '4': ['reverse_speed', 1.1],
        '3': ['reverse_speed', 0.9],
        '6': ['turn_speed', 1.1],
        '5': ['turn_speed', 0.9],
        'I': ['drive_dist', 1.1],
        '<': ['drive_dist', 0.9],
        'L': ['turn_theta', 1.1],
        'J': ['turn_theta', 0.9],
    }
    param_client = dynamic_reconfigure.client.Client(
        rovername + '_MOBILITY',
        config_callback=update_params
    )
    server_config = param_client.get_configuration()
    ignore_obstacles = Obstacle.PATH_IS_CLEAR

    status_msgs = []

    try:
        while True:
            print (msg)
            print (obstacle_msg(ignore_obstacles))
            print (params_msg(
                params['drive_speed'],
                params['reverse_speed'],
                params['turn_speed'],
                params['drive_dist'],
                params['turn_theta']
            ))
            for status in status_msgs:
                print(status)
            status_msgs = []
            key = get_key()
            try:
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
                elif key in obstacle_bindings.keys():
                    if obstacle_bindings[key] == 0:
                        ignore_obstacles = 0
                    else:
                        if (obstacle_bindings[key] & ignore_obstacles
                            == obstacle_bindings[key]):
                            ignore_obstacles ^= obstacle_bindings[key]
                        else:
                            ignore_obstacles |= obstacle_bindings[key]
                else:
                    if (key == '\x03'):
                        break
            except TagException as e:
                status_msgs.append('\033[91m*****I saw a tag!*****\033[0m')
            except HomeException as e:
                status_msgs.append('\033[91m*****I saw Home!*****\033[0m')
            except ObstacleException as e:
                status_msgs.append("\033[91m*****There's an obstacle in front of me*****\033[0m")
            except (PathException, AbortException) as e:
                status_msgs.append('\033[91m*****Exception:*****\033[0m')
                for exception in traceback.format_exception_only(type(e), e):
                    status_msgs.append(exception)

    except Exception as e:
        print('Something went wrong:')
        for exception in traceback.format_exception_only(type(e), e):
            print(exception)
        traceback.print_exc()

    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__=="__main__":
    main()

#! /usr/bin/env python
"""
Modified from ros-teleop/teleop_twist_keyboard.
https://github.com/ros-teleop/teleop_twist_keyboard
Publishes Twists on the rover's /driveControl topic to drive the
rover with keyboard. Twists are unit messages that the driver scales
with the current drive speed and turn speed on the parameter server.
"""
from __future__ import print_function
import sys
import select
import termios
import textwrap
import traceback
import tty
import rospy
import rosnode

import dynamic_reconfigure.client
from geometry_msgs.msg import Twist
from mobility.driver import State
from mobility.swarmie import Swarmie


msg = """
Reading from the keyboard and Publishing Twists to driver!
----------------------------------------------------------
Moving around:       Fingers (use the shift key):
   u    i    o       U         O
   j    k    l       (close)   (open)
   m    ,    .                  

Wrist:               Driving Parameters:
-----------------    -------------------------------------
t : up               1/2 : -/+ drive speed by 10%
g : middle           5/6 : -/+ turn speed by 10%
b : down

anything else : stop

CTRL-C to quit
----------------------------------------------------------
Currently:"""


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def params_msg(drive_speed, turn_speed):
    msg = '''{}: {:.2f} (m/s) | {}: {:.2f} (rad/s)'''
    return textwrap.dedent(msg.format(
        'drive(1/2)',
        drive_speed,
        'turn(5/6)',
        turn_speed
    ))


def update_params(config):
    global params
    params['drive_speed'] = config['DRIVE_SPEED']
    params['turn_speed'] = config['TURN_SPEED']


if __name__ == "__main__":

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

    global swarmie
    swarmie = Swarmie(rovername, node_suffix='_teleop_twist_keyboard')

    settings = termios.tcgetattr(sys.stdin)

    global params
    params = {}

    param_client = dynamic_reconfigure.client.Client(
        rovername + '_MOBILITY',
        config_callback=update_params
    )
    server_config = param_client.get_configuration()


    pub = rospy.Publisher(rovername + '/keyboard', Twist, queue_size=1)

    moveBindings = {
        'i': (1, 0, 0, 0),
        'o': (1, 0, 0, -1),
        'j': (0, 0, 0, 1),
        'l': (0, 0, 0, -1),
        'u': (1, 0, 0, 1),
        ',': (-1, 0, 0, 0),
        '.': (-1, 0, 0, 1),
        'm': (-1, 0, 0, -1),
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
        '6': ['turn_speed', 1.1],
        '5': ['turn_speed', 0.9],
    }

    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print (msg)
        print (params_msg(
            params['drive_speed'],
            params['turn_speed'],
        ))
        while True:
            key = getKey()
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][3]
            elif key in claw_bindings.keys():
                claw_bindings[key]()  # call the function at that key
            elif key in param_bindings.keys():
                params[param_bindings[key][0]] *= param_bindings[key][1]
                if param_bindings[key][0] == 'drive_speed':
                    param_client.update_configuration(
                        {'DRIVE_SPEED': params['drive_speed']}
                    )
                elif param_bindings[key][0] == 'turn_speed':
                    param_client.update_configuration(
                        {'TURN_SPEED': params['turn_speed']}
                    )
                # Update params once now to make sure no params were
                # set to invalid values.
                server_config = param_client.get_configuration()
                update_params(server_config)

                if (status == 14):
                    print (msg)
                print (params_msg(
                    params['drive_speed'],
                    params['turn_speed'],
                ))
                status = (status + 1) % 15
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x
            twist.angular.z = th
            pub.publish(twist)

    except Exception as e:
        print('Something went wrong:')
        for exception in traceback.format_exception_only(type(e), e):
            print(exception)
        traceback.print_exc()

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

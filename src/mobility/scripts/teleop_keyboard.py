#! /usr/bin/env python
"""
Modified from ros-teleop/teleop_twist_keyboard.
https://github.com/ros-teleop/teleop_twist_keyboard
Uses Swarmie API or Joy messages published on the driver's /joystick topic to
drive the rover with keyboard.
"""
from __future__ import print_function

from collections import OrderedDict
import math
import sys
import select
import termios
import textwrap
import traceback
import tty
import argparse

import rospy

import dynamic_reconfigure.client
from sensor_msgs.msg import Joy
from swarmie_msgs.msg import Obstacle
from mobility.swarmie import (swarmie, TagException, HomeException,
                              ObstacleException, PathException, AbortException,
                              InsideHomeException, HomeCornerException)


class Teleop(object):
    """Teleop base class."""

    def __init__(self, rovername):
        # self.swarmie = Swarmie(node_name='teleop_keyboard')

        self.params = {}
        self.param_client = dynamic_reconfigure.client.Client(
            'mobility',
            config_callback=self.update_params
        )
        server_config = self.param_client.get_configuration()
        self.update_params(server_config) # fill rest of params into self.params

        # Keybindings common to both derived classes:
        self.claw_bindings = {
            'O': swarmie.fingers_open,
            'U': swarmie.fingers_close,
            't': swarmie.wrist_up,
            'g': swarmie.wrist_middle,
            'b': swarmie.wrist_down,
        }
        self.param_bindings = {
            '1': ['drive_speed', 0.9],
            '2': ['drive_speed', 1.1],
            '5': ['turn_speed', 0.9],
            '6': ['turn_speed', 1.1],
        }
        self.settings = termios.tcgetattr(sys.stdin)

    def update_params(self, config):
        self.params['drive_speed'] = config['DRIVE_SPEED']
        self.params['reverse_speed'] = config['REVERSE_SPEED']
        self.params['turn_speed'] = config['TURN_SPEED']

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


class TeleopSwarmie(Teleop):
    """Teleop using Swarmie API."""

    msg = '''
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
    --------------------------------------------------------'''

    def __init__(self, rovername):
        super(TeleopSwarmie, self).__init__(rovername)
        self.ignore_obst = Obstacle.PATH_IS_CLEAR
        self.params['drive_dist'] = 0.5
        self.params['turn_theta'] = math.pi / 2

        self.drive_bindings = {
            'i': 1,  # positive
            ',': -1,  # negative
        }
        self.turn_bindings = {
            'j': 1,  # turn left, positive theta
            'l': -1,  # turn right, negative theta
        }
        self.obst_bindings = OrderedDict((
            ('!', ('PATH_IS_CLEAR', Obstacle.PATH_IS_CLEAR)),
            ('a', ('SONAR_LEFT',    Obstacle.SONAR_LEFT)),
            ('s', ('SONAR_CENTER',  Obstacle.SONAR_CENTER)),
            ('d', ('SONAR_RIGHT',   Obstacle.SONAR_RIGHT)),
            ('f', ('SONAR_BLOCK',   Obstacle.SONAR_BLOCK)),
            ('T', ('TAG_TARGET',    Obstacle.TAG_TARGET)),
            ('h', ('TAG_HOME',      Obstacle.TAG_HOME)),
            ('n', ('INSIDE_HOME',   Obstacle.INSIDE_HOME)),
            ('C', ('HOME_CORNER',   Obstacle.HOME_CORNER)),
            ('S', ('IS_SONAR',      Obstacle.IS_SONAR)),
            ('V', ('IS_VISION',     Obstacle.IS_VISION)),
            ('v', ('VISION_SAFE',   Obstacle.VISION_SAFE)),
            ('H', ('VISION_HOME',   Obstacle.VISION_HOME))
        ))
        self.param_bindings['3'] = ['reverse_speed', 0.9]
        self.param_bindings['4'] = ['reverse_speed', 1.1]
        self.param_bindings['I'] = ['drive_dist', 1.1]
        self.param_bindings['<'] = ['drive_dist', 0.9]
        self.param_bindings['L'] = ['turn_theta', 1.1]
        self.param_bindings['J'] = ['turn_theta', 0.9]

    def obstacle_msg(self):
        msg = """
        Toggle Obstacles to ignore (* = currently ignored):
        -----------------
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}

        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}

        ({}) {:<14}= {}{}
        ({}) {:<14}= {}{}
        """
        cur_ignore = []

        if self.ignore_obst == 0:
            cur_ignore = ['*'] + [''] * (len(self.obst_bindings) - 1)
        else:
            cur_ignore.append('')
            for key, obstacle in self.obst_bindings.items()[1:]:
                if obstacle[1] & self.ignore_obst == obstacle[1]:
                    cur_ignore.append('*')
                else:
                    cur_ignore.append(' ')

        result = []

        for (key, obstacle), formatter in zip(self.obst_bindings.items(),
                                              cur_ignore):
            result.append(key)
            result.extend(obstacle)
            result.append(formatter)

        return textwrap.dedent(msg.format(*result))

    def params_msg(self):
        msg = '''Currently:
        {:<20}: {:.2f} (m/s)
        {:<20}: {:.2f} (m/s)
        {:<20}: {:.2f} (rad/s)
        {:<20}: {:.2f} (m)
        {:<20}: {:.2f} (rad)'''
        return textwrap.dedent(msg.format(
            'drive speed (1/2)',
            self.params['drive_speed'],
            'reverse speed (3/4)',
            self.params['reverse_speed'],
            'turn speed (5/6)',
            self.params['turn_speed'],
            'drive dist (I/<)',
            self.params['drive_dist'],
            'turn theta (J/L)',
            self.params['turn_theta']
        ))

    def run(self):
        status_msgs = []
        try:
            while True:
                print (textwrap.dedent(TeleopSwarmie.msg))
                print (self.obstacle_msg())
                print (self.params_msg())
                for status in status_msgs:
                    print(status)
                status_msgs = []
                key = self.get_key()
                try:
                    if key in self.drive_bindings.keys():
                        dist = self.params['drive_dist']
                        if self.drive_bindings[key] < 0:
                            dist = -dist
                        swarmie.drive(dist, ignore=self.ignore_obst)
                    elif key in self.turn_bindings.keys():
                        theta = self.params['turn_theta']
                        if self.turn_bindings[key] < 0:
                            theta = -theta
                        swarmie.turn(theta, ignore=self.ignore_obst)
                    elif key in self.claw_bindings.keys():
                        self.claw_bindings[key]()
                    elif key in self.param_bindings.keys():
                        self.params[self.param_bindings[key][0]]\
                            *= self.param_bindings[key][1]
                        if self.param_bindings[key][0] == 'drive_speed':
                            self.param_client.update_configuration(
                                {'DRIVE_SPEED': self.params['drive_speed']}
                            )
                        elif self.param_bindings[key][0] == 'reverse_speed':
                            self.param_client.update_configuration(
                                {'REVERSE_SPEED': self.params['reverse_speed']}
                            )
                        elif self.param_bindings[key][0] == 'turn_speed':
                            self.param_client.update_configuration(
                                {'TURN_SPEED': self.params['turn_speed']}
                            )
                        # Update params once now to make sure no params were
                        # set to invalid values.
                        server_config = self.param_client.get_configuration()
                        self.update_params(server_config)
                    elif key in self.obst_bindings.keys():
                        if self.obst_bindings[key][1] == 0:
                            self.ignore_obst = 0
                        else:
                            if (self.obst_bindings[key][1] & self.ignore_obst
                                    == self.obst_bindings[key][1]):
                                self.ignore_obst ^= self.obst_bindings[key][1]
                            else:
                                self.ignore_obst |= self.obst_bindings[key][1]
                    else:
                        if (key == '\x03'):
                            break
                except TagException as e:
                    status_msgs.append('\033[91m*****I saw a tag!*****\033[0m')
                except HomeException as e:
                    status_msgs.append('\033[91m*****I saw Home!*****\033[0m')
                except HomeCornerException as e:
                    status_msgs.append(
                        '\033[91m*****I saw a corner of Home!*****\033[0m'
                    )
                except InsideHomeException as e:
                    status_msgs.append(
                        '\033[91m*****I might be inside of Home!*****\033[0m'
                    )
                except ObstacleException as e:
                    status_msgs.append(
                        "\033[91m*****There's an obstacle in front of "
                        + "me*****\033[0m")
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
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


class TeleopKey(Teleop):
    """Teleop using Joy messages published to driver"""

    msg = '''
    Reading from the keyboard and Publishing to driver!
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
    Currently:'''

    def __init__(self, rovername):
        super(TeleopKey, self).__init__(rovername)
        self.pub = rospy.Publisher('joystick', Joy, queue_size=1)
        self.move_bindings = {
            'i': (0, 0, 0, 0, 1, 0),
            'o': (0, 0, 0, -1, 1, 0),
            'j': (0, 0, 0, 1, 0, 0),
            'l': (0, 0, 0, -1, 0, 0),
            'u': (0, 0, 0, 1, 1, 0),
            ',': (0, 0, 0, 0, -1, 0),
            '.': (0, 0, 0, 1, -1, 0),
            'm': (0, 0, 0, -1, -1, 0),
        }

    def params_msg(self):
        msg = '''{}: {:.2f} (m/s) | {}: {:.2f} (rad/s)'''
        return textwrap.dedent(msg.format(
            'drive(1/2)',
            self.params['drive_speed'],
            'turn(5/6)',
            self.params['turn_speed']
        ))

    def run(self):
        joy = Joy()
        joy.axes = (0, 0, 0, 0, 0, 0)
        status = 0
        try:
            print (textwrap.dedent(TeleopKey.msg))
            print (self.params_msg())
            while True:
                key = self.get_key()
                if key in self.move_bindings.keys():
                    joy.axes = self.move_bindings[key]
                elif key in self.claw_bindings.keys():
                    self.claw_bindings[key]()
                elif key in self.param_bindings.keys():
                    self.params[self.param_bindings[key][0]]\
                        *= self.param_bindings[key][1]
                    if self.param_bindings[key][0] == 'drive_speed':
                        self.param_client.update_configuration(
                            {'DRIVE_SPEED': self.params['drive_speed']}
                        )
                    elif self.param_bindings[key][0] == 'turn_speed':
                        self.param_client.update_configuration(
                            {'TURN_SPEED': self.params['turn_speed']}
                        )
                    # Update params once now to make sure no params were
                    # set to invalid values.
                    server_config = self.param_client.get_configuration()
                    self.update_params(server_config)

                    if (status == 14):
                        print (textwrap.dedent(TeleopKey.msg))
                    print (self.params_msg())
                    status = (status + 1) % 15
                else:
                    joy.axes = (0, 0, 0, 0, 0, 0)
                    if (key == '\x03'):
                        break
                self.pub.publish(joy)
        except Exception as e:
            print('Something went wrong:')
            for exception in traceback.format_exception_only(type(e), e):
                print(exception)
            traceback.print_exc()
        finally:
            joy.axes = (0, 0, 0, 0, 0, 0)
            self.pub.publish(joy)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)


def main(use_swarmie=False):
    rovername = rospy.get_namespace().strip('/')
    swarmie.start(node_name='teleop')

    if use_swarmie:
        teleop = TeleopSwarmie(rovername)
    else:
        teleop = TeleopKey(rovername)

    teleop.run()


if __name__== '__main__':
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
        )
    parser.add_argument(
        '-s',
        '--swarmie',
        action='store_true',
        help='use Swarmie API instead to issue driving commands'
    )
    args = parser.parse_args()
    main(use_swarmie=args.swarmie)

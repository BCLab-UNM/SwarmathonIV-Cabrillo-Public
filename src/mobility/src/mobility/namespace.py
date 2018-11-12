#! /usr/bin/env python
"""Helper script to make working with namespaces in ROS a little easier.

This file contains functions that can be imported at the top of behavior
scripts in order to set the ROS_NAMESPACE environment variable in python
code before any ROS modules are imported. After a ROS module is imported,
it's too late to change the environment variable.

It's only helpful to set the ROS_NAMESPACE environment variable this way when
running a behavior script by itself (as opposed to being launched by the task
manager, so the namespace code should only run if a file is being run as the
main program.

This file can currently live in mobility/src/mobility, but only if no ROS
modules are imported in mobility/src/mobility/__init__.py because the
environment variable is read

Examples:
    >>> if __name__ == '__main__':
    >>>     from mobility.namespace import parse_args_and_set_namespace
    >>>     parse_args_and_set_namespace()
    >>>
    >>> # Now it's ok to import ROS stuff and any other modules that use ROS
    >>> import rospy
    >>> import tf
    >>> from mobility.swarmie import Swarmie
    >>> from mobility.planner import Planner

    >>> # If you need to add any command line arguments other than the default
    >>> # optional rovername arg
    >>> # Import the argparse parser object from this module and add to it
    >>> if __name__ == '__main__':
    >>>     from mobility.namespace import parser, set_namespace
    >>>
    >>>    parser.add_argument(
    >>>        '--has-block',
    >>>        action='store_true',
    >>>        help=('whether the rover currently has a block, and should ' +
    >>>              'accordingly either avoid cubes or stop for them')
    >>>    )
    >>>    args = parser.parse_args()
    >>>
    >>>    set_namespace(args.rovername)

"""
from __future__ import print_function

import argparse
import os
import readline
import sys
from subprocess import check_output


# Default global argument parser to parse a single positional arg.
# This parser can be imported by other scripts to add additional flags if
# necessary.
parser = argparse.ArgumentParser(
    formatter_class=argparse.ArgumentDefaultsHelpFormatter
)
parser.add_argument(
    'rovername',
    help='optional, name of the rover to connect to',
    nargs='?',
    default=None
)

class RoverCompleter():
    def __init__(self, rovers):
        """Convenience class to use tab completion in user input for rover
        names.

        Args:
            rovers: the list of available rovers
        """
        self.rovers = rovers

    def complete(self, text, state):
        for rovername in self.rovers:
            if rovername.startswith(text):
                if not state:
                    return rovername
                else:
                    state -= 1


def find_rovers():
    """Get a list of the rovers currently online.

    Returns:
        (list): the list of rovers
    """
    topics = check_output('rostopic list', shell=True).split('\n')
    rovers = set()

    for topic in topics:
        if 'mobility' in topic:
            rover = topic.lstrip('/').split('/')[0]
            rovers.add(rover)

    return list(rovers)


def set_namespace(rovername=None):
    """Set the ROS_NAMESPACE environment variable.

    The environment variable will be set using this priority:

    1. Using the rovername arg, overwriting any value currently set in the
       environment variable for this process.
    2. Using the environment variable's current value (if rovername is None).
    3. Prompting the user to select a rover from a list of rovers currently
       online.

    Args:
        rovername (string): the rover to connect to

    Returns:
        None
    """
    if rovername is not None:
        print('Setting ROS_NAMESPACE={} for this process.'.format(rovername))
        os.environ['ROS_NAMESPACE'] = rovername
        return

    if 'ROS_NAMESPACE' in os.environ:
        print('Using ROS_NAMESPACE={}'.format(os.environ['ROS_NAMESPACE']))
        return

    rovers = find_rovers()

    if len(rovers) == 0:
        print('\033[91m',"No Rovers Detected",'\033[0m', file=sys.stderr)
        sys.exit(-1)
    elif len(rovers) == 1:
        rovername = rovers.pop()
        print('Detected rovers: ', rovername)
        print('\033[92m',"Auto selected:",rovername,'\033[0m')
    else:
        print('Detected rovers:')
        for rover in rovers:
            print(rover)

        rovername = ''
        completer = RoverCompleter(rovers)

        readline.parse_and_bind("tab: complete")
        readline.set_completer(completer.complete)

        print('\nTab completion enabled\n')

        try:
            while rovername not in rovers:
                rovername = raw_input(
                    'Which rover would you like to connect to? '
                )
        except KeyboardInterrupt:
            print('\nGoodbye', file=sys.stderr)
            sys.exit(-1)

    os.environ['ROS_NAMESPACE'] = rovername


def parse_args_and_set_namespace(*args):
    """Parse args from sys.argv and set the ROS_NAMESPACE env variable
    accordingly.

    Examples:
        >>> import sys
        >>> parse_args_and_set_namespace(*sys.argv[1:])
        >>> # wait to import rospy until namespace has been set
        >>> import rospy
        >>> print(rospy.get_namespace())
        /achilles/

    Args:
        *args: args to be parsed, if no args are passed, sys.argv[1:] will be
            parsed.

    Returns:
        None
    """
    if not args:
        args = []
        for a in sys.argv[1:]:
            args.append(a)

    args = parser.parse_args(args)
    set_namespace(args.rovername)


def main():
    """For testing this as a standalone script."""
    parse_args_and_set_namespace()


if __name__ == '__main__':
    main()
    import rospy
    print(rospy.get_namespace())

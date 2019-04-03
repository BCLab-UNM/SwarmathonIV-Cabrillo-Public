#!/usr/bin/env python
PACKAGE = "mobility"

import math 

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
mob = gen.add_group("Mobility")

mob.add("DRIVE_SPEED", double_t, 0, "Default/manual forward driving speed (m/s)", 0.3, 0.1, 0.6)
mob.add("REVERSE_SPEED", double_t, 0, "Backward driving speed (m/s)", 0.2, 0.1, 0.3)
mob.add("TURN_SPEED", double_t, 0, "Default/manual turning speed (r/s)", 0.6, 0.1, 1.2)
mob.add("HEADING_RESTORE_FACTOR", double_t, 0, "Gives turn speed (in r/s) from heading error (r).", 2, 0, 5)
mob.add("GOAL_DISTANCE_OK", double_t, 0, "Distance when we're considered at the goal.", 0.1, 0, math.pi)
mob.add("ROTATE_THRESHOLD", double_t, 0, "Goal angle for the turn state.", math.pi / 16, 0, math.pi)
mob.add("DRIVE_ANGLE_ABORT", double_t, 0, "Angle where we abort the drive state.", math.pi / 4, 0, math.pi)

swarmie = gen.add_group("Swarmie")
swarmie.add("DRIVE_SPEED_SLOW", double_t, 0, "Slow forward driving speed (m/s)", 0.15, 0.1, 0.6)
swarmie.add("TURN_SPEED_SLOW", double_t, 0, "Slow turning speed (r/s)", 0.4, 0.1, 1.2)
swarmie.add("DRIVE_SPEED_FAST", double_t, 0, "Fast forward driving speed (m/s)", 0.3, 0.1, 0.6)
swarmie.add("TURN_SPEED_FAST", double_t, 0, "Fast turning speed (r/s)", 0.7, 0.1, 1.2)

exit(gen.generate(PACKAGE, "config", "drive"))

#!/usr/bin/env python
PACKAGE = "mobility"

import math 

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
mob = gen.add_group("Mobility")

mob.add("DRIVE_SPEED_SLOPE", double_t, 0, "Slope of the speed vs. distance line.", 1, 0, 5)
mob.add("DRIVE_SPEED_MIN", double_t, 0, "Minimum linear speed.", 0.1, 0, 1)
mob.add("DRIVE_SPEED_MAX", double_t, 0, "Maximum linear speed.", 0.5, 0, 1)
    
mob.add("TURN_SPEED_SLOPE", double_t, 0, "Slope of the speed vs. radians line.", 2.0, 0, 5);
mob.add("TURN_SPEED_MIN", double_t, 0, "Minimum turning speed.", 0.1, 0, 1);
mob.add("TURN_SPEED_MAX", double_t, 0, "Maximum turning speed.", 0.7, 0, 1);
    
mob.add("GOAL_DISTANCE_OK", double_t, 0, "Distance when we're considered at the goal.", 0.1, 0, math.pi);

mob.add("ROTATE_THRESHOLD", double_t, 0, "Goal angle for the turn state.", math.pi / 16, 0, math.pi);
mob.add("DRIVE_ANGLE_ABORT", double_t, 0, "Angle where we abort the drive state.", math.pi / 2, 0, math.pi);

mob.add("STOP_THRESHOLD", double_t, 0, "Speed below which we're considered stopped.", 0.1, 0, 1);

exit(gen.generate(PACKAGE, "config", "drive"))

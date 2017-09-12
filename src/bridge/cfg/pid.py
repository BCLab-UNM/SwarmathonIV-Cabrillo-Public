#!/usr/bin/env python
PACKAGE = "bridge"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()
pid = gen.add_group("PID")

pid.add("scale", double_t, 0, "Scale factor for PID coefficients.", 0, 0, 1000)
pid.add("Kp", double_t, 0, "Kp for wheel velocity.", 0, 0, 1)
pid.add("Ki", double_t, 0, "Ki for wheel velocity.", 0, 0, 1)
pid.add("Kd", double_t, 0, "Kd for wheel velocity.", 0, 0, 1)
pid.add("db", double_t, 0, "Deadband for wheel velocity.", 0, 0, 128)
pid.add("st", double_t, 0, "Stiction for wheel velocity.", 0, 0, 128)
pid.add("wu", double_t, 0, "Windup limit wheel velocity.", 0, 0, 65535)
pid.add("ff", double_t, 0, "Feed forward slope.", 0, 0, 65535)

exit(gen.generate(PACKAGE, "pid_dynconfig", "pid"))

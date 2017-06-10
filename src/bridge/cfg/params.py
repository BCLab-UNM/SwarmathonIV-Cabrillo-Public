#!/usr/bin/env python
PACKAGE = "bridge"

from dynamic_reconfigure.parameter_generator_catkin import *

gen = ParameterGenerator()

gen.add("linear_Kp", double_t, 0, "Kp for linear velocity.", 0.01, 0, 1)
gen.add("linear_Ki", double_t, 0, "Ki for linear velocity.", 0, 0, 1)
gen.add("linear_Kd", double_t, 0, "Kd for linear velocity.", 0, 0, 1)
gen.add("linear_db", double_t, 0, "Deadband for linear velocity.", 0, 0, 128)
gen.add("linear_st", double_t, 0, "Stiction for linear velocity.", 0, 0, 128)
gen.add("linear_wu", double_t, 0, "Windup limit linear velocity.", 1024, 0, 65535)

gen.add("angular_Kp", double_t, 0, "Kp for angular velocity.", 0.01, 0, 1)
gen.add("angular_Ki", double_t, 0, "Ki for angular velocity.", 0, 0, 1)
gen.add("angular_Kd", double_t, 0, "Kd for angular velocity.", 0, 0, 1)
gen.add("angular_db", double_t, 0, "Deadband for angular velocity.", 0, 0, 128)
gen.add("angular_st", double_t, 0, "Stiction for angular velocity.", 0, 0, 128)
gen.add("angular_wu", double_t, 0, "Windup limit angular velocity.", 1024, 0, 65535)

exit(gen.generate(PACKAGE, "config", "Drive"))

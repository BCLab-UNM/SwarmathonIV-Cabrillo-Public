int8 BOUNDARY_OPT_UNSET = -1
int8 BOUNDARY_OPT_1 = 1
int8 BOUNDARY_OPT_2 = 2

string rover_name

# A rover's start time is the primary method for determining authority.
time start_time

# A randomly generated priority number is the secondary method for determining
# authority, if 2 or more rovers have the same start time.
uint64 random_priority

# Which option this rover wants to use.
int8 boundary_option

# The angle by which the bounds have been rotated. The bounds are rotated to
# put each corner's heading at their center, so they're applicable to other
# rovers, taking small compass errors into account.
float64 boundary_theta
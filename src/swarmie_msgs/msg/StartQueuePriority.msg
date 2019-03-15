uint8 QUEUE_ONE = 0
uint8 QUEUE_TWO = 1

string rover_name

# Which queue the rover should be placed into. Determined by the rover's initial
# heading. Positive headings correspond to queue_index 0, and negative headings
# correspond to queue_index 1.
uint8 queue_index

# What priority the rover should be given in the queue. Priorities are determined
# using a combination of the rover's initial heading and a numerical priority
# penalty if the rover needs to do a 2D IMU calibration. This way, the queue
# generally progresses in the order the rovers are arranged around the home ring,
# while putting rovers that need an IMU calibration and will spin in place for
# 20-30 sec at the end of the queue.
float64 priority

# Whether this rover has already waited in the start queue. This is used to tell
# a rover brought up mid-round that it doesn't need to add this rover to its
# start queue. This is more reliable than trying to unregister a publisher on
# the queue priorities topic once a rover has finished queueing.
bool is_finished

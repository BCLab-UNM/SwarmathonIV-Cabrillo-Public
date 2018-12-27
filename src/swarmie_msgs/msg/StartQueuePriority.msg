uint8 QUEUE_ONE = 0
uint8 QUEUE_TWO = 1

string rover_name

# Which queue the rover should be placed into. Determined by the rover's initial
# heading. Positive headings correspond to queue_index 0, and negative headings
# correspond to queue_index 1.
uint8 queue_index

# What priority the rover should be given in the queue.
float64 priority
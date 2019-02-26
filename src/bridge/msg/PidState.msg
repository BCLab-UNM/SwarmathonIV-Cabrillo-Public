# Summary data for the rover itself and both wheels individually
std_msgs/Header header

float64 linear_setpoint # rover (encoder ticks/sec)
float64 angular_setpoint # rover (encoder ticks/sec)

bridge/PidWheel left_wheel
bridge/PidWheel right_wheel

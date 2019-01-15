# per wheel data

# current commanded linear speed +/- angular speed (encoder ticks/sec)
float64 setpoint
float64 feedback # current wheel speed (encoder ticks/sec)

float64 output # current output sent to the arduino
float64 error # setpoint - ticks
float64 p_term
float64 i_term
float64 d_term

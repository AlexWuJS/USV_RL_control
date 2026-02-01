# 'S'报文
bool push_is_open
bool telemetry_enabled
bool tcp_is_connecting
bool is_auto_control
bool is_keyboard_control
bool loop_mode_enabled
# 'M'报文
int32 control_mode
bool is_armed
bool is_moving
bool is_station_keeping
# 'P'报文

# 'R'报文
float64 longitude
float64 latitude
float64 heading_angle
float64 voltage_left_battery
float64 voltage_right_battery
float64 temperature

# 'K' 报文 摇杆控制
tcp/joystick joystick

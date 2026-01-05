# 'S'报文
bool push_is_open
bool ship_is_sending
bool tcp_is_connecting
# char control_mode
bool is_auto_control
bool is_keyboard_control
bool boat_keep_nav
# 'M'报文
bool boat_is_active
bool boat_is_running
bool boat_is_smart_remote_mode
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

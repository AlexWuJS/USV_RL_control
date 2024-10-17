# 'S'报文
bool push_is_open
bool ship_is_sending
bool tcp_is_connecting
char control_mode
bool boat_keep_nav
# 'M'报文
char boat_is_active
char boat_is_running
char boat_is_smart_mode
# 'P'报文

# 'R'报文
float64 longitude
float64 latitude
float64 heading_angle
float64 voltage_left_battery
float64 voltage_right_battery
float64 temperature

# 'K' 报文 键盘控制
tcp/keyboard keyboard

# 'X'报文
# tcp/pid PID1
# tcp/pid PID2
# tcp/pid PID3

# tcp/system_identification system_identification # 系统辨识

# tcp/sliding_mode sliding_mode     # 滑模控制
# guidance 层输出指令（LOS 或 RL 产生，经 Safety Supervisor 后由 huamo 执行）
# desired_course 一律为 camera_init 系标准 ENU 航迹向，单位 rad，范围 (-pi, pi]
# huamo 执行段换算：desired_heading = normalizeAngle(desired_course + pi)
std_msgs/Header header
float64 desired_speed      # m/s, [0, v_max]；0.0 = 停船
float64 desired_course     # rad, camera_init 系 ENU 航迹向
uint8  source              # 0=LOS 1=RL 2=SUPERVISOR_OVERRIDE（见 rl_common/contracts.py）
uint8  supervisor_active   # 1=本拍 Safety Supervisor 介入过（仅诊断用）

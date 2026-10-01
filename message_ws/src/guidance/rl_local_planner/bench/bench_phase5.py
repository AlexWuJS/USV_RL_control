#!/usr/bin/env python3
"""Phase 5 台架验证 v2：带原始 L/R 诊断 + 容错 + 反符号看门狗断言。"""
import math, time
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from rl_local_planner.msg import NavCmd
from thruster_manager.msg import Thruster

A, B = 0.793023, 0.0236265
k1, k2, a1, a2, a3, b1, b2 = 1.7, 0.1, -0.66, 0.5, 3, 0.4, 0.1
def expect_fwd_pwm(sp):
    if sp <= 0: return 1500.0
    s = -10.0*sp
    reach = k1*abs(s)**a1*math.tanh(b1*s) + k2*abs(s)**a2*math.asinh(b2*(s**3))  # s**3 保号（原式 pow(s,3)）
    u = -A/B*reach
    u = max(0.0, min(100.0, u))
    # remote_data_deal: left>1 时 PWM=1550+2.5*left（+50 偏置），|left|<=1 时 1500
    return 1500.0 + (50.0 + 2.5*u if u > 1 else 0.0)

rospy.init_node("bench_phase5", anonymous=True)
import tf2_ros
from geometry_msgs.msg import TransformStamped
tf_br = tf2_ros.TransformBroadcaster()
boat_pos = [0.0, 0.0]
def publish_tf(x, y):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "camera_init"; t.child_frame_id = "body"
    t.transform.translation.x = x; t.transform.translation.y = y
    t.transform.rotation.w = 1.0
    tf_br.sendTransform(t)
od = rospy.Publisher("/Odometry", Odometry, queue_size=1)
pp = rospy.Publisher("/my_path", Path, queue_size=1)
rc = rospy.Publisher("/guidance/rl_cmd", NavCmd, queue_size=1)
pwms = []
def cb(m): pwms.append((time.time(), m.left_thruster_pwm, m.right_thruster_pwm))
rospy.Subscriber("/auto_thruster_pwm", Thruster, cb)

path = Path(); path.header.frame_id = "camera_init"
for i in range(41):
    p = PoseStamped(); p.header.frame_id = "camera_init"
    p.pose.position.x = i*0.5; p.pose.orientation.w = 1.0
    path.poses.append(p)
odom = Odometry(); odom.header.frame_id = "camera_init"
odom.pose.pose.position.z = 0.0; odom.pose.pose.orientation.w = 1.0

r = rospy.Rate(50)
t0 = time.time()
def publish_odom():
    odom.header.stamp = rospy.Time.now()
    odom.pose.pose.position.x = boat_pos[0]
    odom.pose.pose.position.y = boat_pos[1]
    publish_tf(boat_pos[0], boat_pos[1])
    od.publish(odom)
def rl(sp, course):
    m = NavCmd(); m.header.stamp = rospy.Time.now()
    m.desired_speed = sp; m.desired_course = course
    m.source = 1; m.supervisor_active = 0
    rc.publish(m)

COURSE0 = math.radians(-88.5)
def stage(seconds, sp=0.0, course=0.0, rl_on=True):
    t_end = time.time() + seconds
    while time.time() < t_end and not rospy.is_shutdown():
        publish_odom()
        if rl_on: rl(sp, course)
        pp.publish(path)
        r.sleep()
def stats(seconds):
    seg = [p for p in pwms if p[0] >= time.time()-seconds+0.6]
    print("    [窗口 %d 条] 首3: %s  末3: %s"
          % (len(seg),
             [(round(x[0]%1000,2), x[1], x[2]) for x in seg[:3]],
             [(round(x[0]%1000,2), x[1], x[2]) for x in seg[-3:]]))
    if len(seg) < 5:
        return 0.0, 0.0, None, None
    L = sorted(x[1] for x in seg); R = sorted(x[2] for x in seg)
    Lm, Rm = L[len(L)//2], R[len(R)//2]
    return Lm, Rm, (Lm+Rm)/2-1500.0, (Lm-Rm)/2.0

print("== 预热 2s（LOS）=="); t1 = time.time()
while time.time() < t1+2.0:
    publish_odom(); pp.publish(path); r.sleep()

res_ok = True
print("== 1 速度阶跃（course=-88.5°, e 应≈0）==")
for sp in (0.2, 0.5, 0.9):
    stage(2.5, sp=sp, course=COURSE0)
    Lm, Rm, fwd, df = stats(2.5)
    if fwd is None:
        print("  sp=%.1f: 无 PWM 消息  FAIL" % sp); res_ok = False; continue
    want = expect_fwd_pwm(sp) - 1500.0
    ok = abs(fwd - want) < 25 and abs(df) < 15
    res_ok &= ok
    print("  sp=%.1f: L=%4.0f R=%4.0f  fwd=%.1f diff=%+.1f  期望fwd=%.1f  %s"
          % (sp, Lm, Rm, fwd, df, want, "PASS" if ok else "FAIL"))

print("== 2 航向 ±30°（speed=0.5）==")
stage(2.0, sp=0.5, course=COURSE0)
Lm, Rm, f0, d0 = stats(2.0)
stage(2.0, sp=0.5, course=COURSE0 + math.radians(30))
Lm, Rm, f1, d1 = stats(2.0)
stage(2.0, sp=0.5, course=COURSE0 - math.radians(30))
Lm, Rm, f2, d2 = stats(2.0)
ok2 = (d1 is not None and d2 is not None and
       ((d1 > 50 and d2 < -50) or (d1 < -50 and d2 > 50)))
res_ok &= ok2
print("  e=0: diff=%s | +30°: %s | -30°: %s  %s"
      % (d0, d1, d2, "PASS" if ok2 else "FAIL"))
# 转向摆率限幅验证（turn 限幅 50/s → diff 限幅 125/s）
seg2 = [p for p in pwms if p[0] >= time.time()-6.0]
slew = []
for i in range(1, len(seg2)):
    dt = seg2[i][0] - seg2[i-1][0]
    if dt > 1e-3:
        slew.append(abs(((seg2[i][2]-seg2[i][1]) - (seg2[i-1][2]-seg2[i-1][1]))/2.0)/dt)
if slew:
    smax = max(slew)
    ok_slew = smax < 200
    res_ok &= ok_slew
    print("  转向摆率: max %.0f PWM/s (限幅125)  %s" % (smax, "PASS" if ok_slew else "FAIL"))

print("== 3 急停（speed=0）==")
stage(2.0, sp=0.0, course=COURSE0)
Lm, Rm, fwd0, df0 = stats(2.0)
ok3 = fwd0 is not None and abs(fwd0) < 8 and abs(df0) < 15
res_ok &= ok3
print("  L=%4.0f R=%4.0f  fwd=%.1f diff=%+.1f  %s" % (Lm, Rm, fwd0, df0, "PASS" if ok3 else "FAIL"))

from std_msgs.msg import Empty as EmptyMsg
hov = rospy.Publisher("/guidance/hover_set", EmptyMsg, queue_size=1)

print("== 3b 悬停进入（艇在锚点）→ 应保持 1500 ==")
# 连续发布 0.5s 建立连接（Publisher 刚创建时首条消息可能丢失）
t_c = time.time()
while time.time() < t_c + 0.5:
    hov.publish(EmptyMsg()); r.sleep()
t_h = time.time()
while time.time() < t_h + 3.0:
    publish_odom(); hov.publish(EmptyMsg()); r.sleep()   # 悬停期间不发路径（会取消悬停）
Lm, Rm, fwd_h, df_h = stats(2.0)
ok_h0 = fwd_h is not None and abs(fwd_h) < 8 and abs(df_h) < 15
res_ok &= ok_h0
print("  进入悬停(在锚点): L=%4.0f R=%4.0f  %s" % (Lm, Rm, "PASS" if ok_h0 else "FAIL"))

print("== 3c 悬停：艇被吹走 3m → 应驱动返回 ==")
boat_pos[0], boat_pos[1] = 3.0, 2.0
t_d = time.time()
while time.time() < t_d + 4.0:
    publish_odom(); hov.publish(EmptyMsg()); r.sleep()
Lm, Rm, fwd_d, df_d = stats(2.5)
resp = fwd_d is not None and (abs(df_d) > 30 or abs(fwd_d) > 15)
res_ok &= resp
print("  被吹走后: L=%4.0f R=%4.0f (fwd=%s diff=%s) → %s"
      % (Lm, Rm, fwd_d, df_d, "PASS(有回驱响应)" if resp else "FAIL"))

print("== 3d 悬停退出：新任务到达 ==")
pp.publish(path)
t_e = time.time()
while time.time() < t_e + 2.0:
    publish_odom(); r.sleep()
seg3 = [p for p in pwms if p[0] >= time.time()-1.5]
exit_ok = len(seg3) > 0
res_ok &= exit_ok
print("  新任务后 PWM 恢复发布: %s" % ("PASS" if exit_ok else "FAIL"))

print("== 4 看门狗（rl 取反符号差速 → 断流应翻转）==")
stage(1.5, sp=0.5, course=COURSE0 - math.radians(60))
Lm, Rm, f_rl, d_rl = stats(1.5)
t_cut = time.time()
while time.time() < t_cut + 2.5:
    publish_odom(); pp.publish(path); r.sleep()
Lm, Rm, f_fb, d_fb = stats(2.0)
ok4 = (d_rl is not None and d_fb is not None and
       ((d_rl > 50 and d_fb < -50) or (d_rl < -50 and d_fb > 50)))
res_ok &= ok4
print("  断流前(RL) diff=%s → 断流后(LOS) diff=%s  %s" % (d_rl, d_fb, "PASS" if ok4 else "FAIL"))

print("===> 总体:", "PASS" if res_ok else "HAS FAIL")

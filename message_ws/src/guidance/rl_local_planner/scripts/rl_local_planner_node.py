#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""rl_local_planner_node — RL 局部规划器 ROS 适配层（RL_INTERFACE_DESIGN.md §4）。

职责边界：
  - 只做"ROS 数据 ↔ rl_common 契约"的转换 + Actor 推理 + Shadow 日志；
  - 观测构建/动作积分/安全规则全部调用 rl_common（训练端共用同一实现）；
  - 永远不发布 /auto_thruster_pwm；控制权在 huamo_node（guidance_mode 选择）。

模式：
  shadow（默认）— 计算并发布 /guidance/rl_cmd，但 huamo 仍用 LOS（不参与控制）
  active        — 同上，且 huamo guidance_mode:=rl 时指令进入执行段

降级行为（不依赖任何东西也能跑起来）：
  - NavCmd 消息未 catkin 构建过 → 只发布诊断状态，不发指令
  - tcp/ship_info 不可用 → 视为自主模式（参数 always_compute 可强制）
  - model_path 为空 / torch 缺失 / 模型加载失败 → STUB actor（零动作）
"""

import os
import sys
import csv
import time
import threading

import numpy as np

# rl_common 与本脚本同包（scripts/ 的上一级）
_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, ".."))

from rl_common import (  # noqa: E402
    BodyState, NavCommand, SRC_RL,
    ActionLimits, ActionProcessor,
    SafetyConfig, SafetySupervisor,
    PathFeatures, LidarSectorizer,
    ObservationBuilder, ObsConfig,
    wrap_scalar_to_pi,
)

import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from nav_msgs.msg import Odometry, Path  # noqa: E402
from sensor_msgs.msg import PointCloud2  # noqa: E402
from sensor_msgs import point_cloud2 as pc2  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402

# --- 可选依赖：缺失时降级，不阻塞节点启动 ---------------------------------
try:
    from rl_local_planner.msg import NavCmd as NavCmdMsg
    HAVE_NAVCMD = True
except ImportError:
    NavCmdMsg, HAVE_NAVCMD = None, False

try:
    from tcp.msg import ship_info as ShipInfoMsg
    HAVE_SHIP_INFO = True
except ImportError:
    ShipInfoMsg, HAVE_SHIP_INFO = None, False

try:
    import torch
    HAVE_TORCH = True
except ImportError:
    torch, HAVE_TORCH = None, False


# ===========================================================================
class StubActor:
    """占位策略：恒零动作 → 参考保持重锚定初值。仅用于打通 shadow 数据链。"""

    def __call__(self, obs):
        return np.zeros(2, dtype=np.float32)


class TorchActor:
    """TorchScript Actor（训练机导出的 actor.pt），仅推理。"""

    def __init__(self, path):
        self.model = torch.jit.load(path, map_location="cpu")
        self.model.eval()

    def __call__(self, obs):
        with torch.no_grad():
            t = torch.from_numpy(obs.astype(np.float32)).unsqueeze(0)
            return self.model(t).squeeze(0).cpu().numpy()


# ===========================================================================
class RLPlannerNode:

    def __init__(self):
        p = rospy.get_param
        # --- node ---
        self.mode = p("~node/mode", "shadow")
        self.always_compute = bool(p("~node/always_compute", False))
        model_path = p("~node/model_path", "")
        log_dir = p("~node/log_dir", "/tmp/rl_logs")
        # --- frames ---
        self.world_frame = p("~frames/world_frame", "camera_init")
        self.map_frame = p("~frames/map_frame", "map")
        self.hull_to_sensor_yaw = float(p("~frames/hull_to_sensor_yaw", 0.0))
        # --- topics ---
        t = p("~topics", {})
        # --- action / safety / obs ---
        self.action_processor = ActionProcessor(ActionLimits(
            dv_max=float(p("~action/dv_max", 0.10)),
            dcourse_max=float(p("~action/dcourse_max", 0.05)),
            v_min=float(p("~action/v_min", 0.2)),
            v_max=float(p("~action/v_max", 2.0)),
        ))
        self.supervisor = SafetySupervisor(SafetyConfig(
            slow_zone=float(p("~safety/slow_zone", 8.0)),
            crawl_zone=float(p("~safety/crawl_zone", 3.0)),
            critical_zone=float(p("~safety/critical_zone", 1.5)),
            crawl_speed=float(p("~safety/crawl_speed", 0.4)),
            course_dev_max=float(p("~safety/course_dev_max", 1.047)),
            v_min=float(p("~action/v_min", 0.2)),
            v_max=float(p("~action/v_max", 2.0)),
        ))
        self.obs_builder = ObservationBuilder(ObsConfig(
            la1_dist=float(p("~obs/la1_dist", 6.0)),
            la2_dist=float(p("~obs/la2_dist", 15.0)),
            n_sectors=int(p("~obs/n_sectors", 36)),
            lidar_max_range=float(p("~obs/lidar_max_range", 20.0)),
        ))
        self.sectorizer = LidarSectorizer(
            n_sectors=int(p("~obs/n_sectors", 36)),
            max_range=float(p("~obs/lidar_max_range", 20.0)),
            h_min=float(p("~obs/h_min", -0.3)),
            h_max=float(p("~obs/h_max", 1.0)),
        )

        # --- actor ---
        self.actor, self.actor_kind = self._load_actor(model_path)

        # --- state ---
        self.lock = threading.Lock()
        self.state = BodyState()
        self.have_odom = False
        self.path_features = None
        self.points_body = np.zeros((0, 3))
        self.is_auto = True
        self.last_raw_action = np.zeros(2)
        self.last_step_time = time.time()
        self.abnormal_since = None

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- pubs / subs ---
        self.rl_cmd_pub = rospy.Publisher(t.get("rl_cmd", "/guidance/rl_cmd"),
                                          NavCmdMsg, queue_size=1) if HAVE_NAVCMD else None
        self.status_pub = rospy.Publisher(t.get("rl_status", "/guidance/rl_status"),
                                          Float32MultiArray, queue_size=1)
        rospy.Subscriber(t.get("odom", "/Odometry"), Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(t.get("path", "/waypoints_path"), Path, self.path_cb, queue_size=2)
        rospy.Subscriber(t.get("cloud", "/cloud_registered_body"), PointCloud2,
                         self.cloud_cb, queue_size=1)
        if HAVE_SHIP_INFO:
            rospy.Subscriber(t.get("ship_info", "/ship/ship_info"), ShipInfoMsg,
                             self.ship_info_cb, queue_size=1)
        rospy.loginfo("rl_local_planner: mode=%s actor=%s navcmd=%s ship_info=%s",
                      self.mode, self.actor_kind, HAVE_NAVCMD, HAVE_SHIP_INFO)

        # --- 运行日志（shadow / active 都记录，文件名带模式前缀） ---
        self.csv_file = None
        self.csv_writer = None
        self._open_log(log_dir)

    # ------------------------------------------------------------------
    def _load_actor(self, model_path):
        if not model_path:
            return StubActor(), "STUB"
        if not HAVE_TORCH:
            rospy.logwarn("torch 不可用，回退 STUB actor")
            return StubActor(), "STUB"
        if not os.path.isfile(model_path):
            rospy.logwarn("模型不存在: %s，回退 STUB actor", model_path)
            return StubActor(), "STUB"
        try:
            return TorchActor(model_path), "TorchScript"
        except Exception as e:  # noqa: BLE001
            rospy.logerr("模型加载失败: %s，回退 STUB actor", e)
            return StubActor(), "STUB"

    def _open_log(self, log_dir):
        try:
            os.makedirs(log_dir, exist_ok=True)
            path = os.path.join(
                log_dir, "%s_%s.csv" % (self.mode, time.strftime("%Y%m%d_%H%M%S")))
            self.csv_file = open(path, "w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["t", "u", "v", "r", "cte", "e_heading", "d_min",
                 "v_ref", "course_ref", "cmd_speed", "cmd_course",
                 "supervisor", "infer_ms", "actor"])
            rospy.loginfo("运行日志(%s): %s", self.mode, path)
        except OSError as e:
            rospy.logwarn("shadow 日志打开失败: %s", e)
            self.csv_file = None

    # ------------------------------------------------------------------
    # 回调
    # ------------------------------------------------------------------
    def ship_info_cb(self, msg):
        self.is_auto = bool(msg.is_auto_control)

    def cloud_cb(self, msg):
        try:
            pts = np.array(list(pc2.read_points(msg, field_names=("x", "y", "z"),
                                                skip_nans=True)), dtype=np.float64)
        except Exception:  # noqa: BLE001
            return
        if pts.size == 0:
            self.points_body = np.zeros((0, 3))
            return
        # 传感器体轴 → 统一艇体体轴（hull_to_sensor_yaw，Phase 5 标定）
        a = self.hull_to_sensor_yaw
        c, s = np.cos(a), np.sin(a)
        xy = pts[:, :2] @ np.array([[c, -s], [s, c]]).T
        self.points_body = np.column_stack([xy, pts[:, 2]])

    def path_cb(self, msg):
        """nav_msgs/Path → camera_init 系点数组 → 重建 PathFeatures + 重锚定参考。"""
        src_frame = msg.header.frame_id or self.map_frame
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, src_frame,
                                                 rospy.Time(0), rospy.Duration(0.2))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(2.0, "路径 TF %s→%s 失败: %s", src_frame,
                                   self.world_frame, e)
            return
        tr = tf.transform.translation
        q = tf.transform.rotation
        yaw = wrap_scalar_to_pi(
            np.arctan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y ** 2 + q.z ** 2)))
        c, s = np.cos(yaw), np.sin(yaw)

        # p_camera_init = R(yaw)·p_map + t
        pts = [(c * ps.pose.position.x - s * ps.pose.position.y + tr.x,
                s * ps.pose.position.x + c * ps.pose.position.y + tr.y)
               for ps in msg.poses]
        if len(pts) < 2:
            self.path_features = None
            return
        with self.lock:
            self.path_features = PathFeatures(
                pts, la1_dist=self.obs_builder.cfg.la1_dist,
                la2_dist=self.obs_builder.cfg.la2_dist)
            # 重锚定：软起步（v_ref=当前 u），course_ref=起点切线
            v0 = self.state.u if self.have_odom else 0.0
            self.action_processor.reset(v0, self.path_features.tangent_at_start())
        rospy.loginfo("收到新路径: %d 点, 弧长 %.1f m", len(pts), self.path_features.s_total)

    def odom_cb(self, msg):
        yaw = wrap_scalar_to_pi(np.arctan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                   msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1.0 - 2.0 * (msg.pose.pose.orientation.y ** 2 +
                         msg.pose.pose.orientation.z ** 2)))
        psi = wrap_scalar_to_pi(yaw + self.hull_to_sensor_yaw)
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        c, s = np.cos(psi), np.sin(psi)
        u = c * vx + s * vy          # 世界速度 → 体轴投影（huamo 同款公式）
        v = -s * vx + c * vy
        with self.lock:
            self.state = BodyState(
                x=msg.pose.pose.position.x, y=msg.pose.pose.position.y,
                psi=psi, u=u, v=v, r=msg.twist.twist.angular.z)
            self.have_odom = True
        self.step()

    # ------------------------------------------------------------------
    # 主步骤（/Odometry 驱动，~10Hz）
    # ------------------------------------------------------------------
    def step(self):
        now = time.time()
        with self.lock:
            state = self.state
            path_features = self.path_features
            points = self.points_body.copy()
            auto = self.is_auto or self.always_compute

        if path_features is None or not path_features.valid or not state or not auto:
            return

        feat = path_features.update_full([state.x, state.y], state.psi)
        sectors_m = self.sectorizer.process(points)
        d_min = self.sectorizer.min_clearance(sectors_m)

        obs = self.obs_builder.build(state, feat, sectors_m,
                                     v_ref=self.action_processor.v_ref,
                                     prev_action=self.last_raw_action)

        # 推理
        t0 = time.time()
        try:
            raw = np.asarray(self.actor(obs), dtype=np.float64).reshape(-1)[:2]
        except Exception as e:  # noqa: BLE001
            rospy.logerr_throttle(1.0, "推理异常: %s", e)
            raw = None
        infer_ms = (time.time() - t0) * 1000.0

        if raw is None or not np.isfinite(raw).all() or not np.isfinite(obs).all():
            self._handle_abnormal(feat.tangent_course, now)
            return
        self.abnormal_since = None
        self.last_raw_action = np.clip(raw, -1.0, 1.0)

        # 动作积分 + Safety Supervisor
        out = self.action_processor.process(self.last_raw_action)
        cmd = NavCommand(desired_speed=out.v_ref, desired_course=out.course_ref,
                         source=SRC_RL)
        cmd = self.supervisor.apply(cmd, d_min, feat.tangent_course)

        # 发布 + 记录
        self._publish_cmd(cmd)
        self._log(now, state, feat, d_min, out, cmd, infer_ms)
        self._publish_status(state, feat, d_min, cmd, infer_ms)

    def _handle_abnormal(self, tangent_course, now):
        """P1 看门狗：短暂异常冻结参考，持续异常急停。"""
        hold = rospy.get_param("~safety/emergency_hold_ms", 1000) / 1000.0
        if self.abnormal_since is None:
            self.abnormal_since = now
            rospy.logwarn("观测/推理异常，冻结参考")
        elif now - self.abnormal_since > hold:
            self._publish_cmd(self.supervisor.emergency_stop(tangent_course))
            rospy.logerr_throttle(1.0, "异常持续 %.1fs，Safety 急停", now - self.abnormal_since)

    # ------------------------------------------------------------------
    def _publish_cmd(self, cmd):
        if self.rl_cmd_pub is None:
            return
        m = NavCmdMsg()
        m.header.stamp = rospy.Time.now()
        m.desired_speed = float(cmd.desired_speed)
        m.desired_course = float(cmd.desired_course)
        m.source = int(cmd.source)
        m.supervisor_active = 1 if cmd.supervisor_active else 0
        self.rl_cmd_pub.publish(m)

    def _publish_status(self, state, feat, d_min, cmd, infer_ms):
        msg = Float32MultiArray()
        msg.data = [
            state.u, state.r, feat.cte,
            wrap_scalar_to_pi(feat.tangent_course - state.course),
            d_min, self.action_processor.v_ref, self.action_processor.course_ref,
            cmd.desired_speed, cmd.desired_course,
            1.0 if cmd.supervisor_active else 0.0, infer_ms,
            1.0 if self.actor_kind == "STUB" else 0.0,
        ]
        self.status_pub.publish(msg)

    def _log(self, now, state, feat, d_min, out, cmd, infer_ms):
        if self.csv_writer is None:
            return
        e_head = wrap_scalar_to_pi(feat.tangent_course - state.course)
        self.csv_writer.writerow([
            "%.3f" % now, "%.4f" % state.u, "%.4f" % state.v, "%.4f" % state.r,
            "%.4f" % feat.cte, "%.4f" % e_head, "%.4f" % d_min,
            "%.4f" % out.v_ref, "%.4f" % out.course_ref,
            "%.4f" % cmd.desired_speed, "%.4f" % cmd.desired_course,
            int(cmd.supervisor_active), "%.2f" % infer_ms, self.actor_kind])
        self.csv_file.flush()

    # ------------------------------------------------------------------
    def spin(self):
        rospy.spin()
        if self.csv_file:
            self.csv_file.close()


if __name__ == "__main__":
    rospy.init_node("rl_local_planner")
    RLPlannerNode().spin()

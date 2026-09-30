#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz 多点导航交互 v2：Publish Point=途经点（图外=清除），2D Nav Goal=终点并即时规划。

交互规则（2026-09 定稿）:
  - "Publish Point" 点图内            → 添加途经点（带编号标记，再点同位置=覆盖修改）
  - "Publish Point" 点**地图边界外**   → 清除整条轨迹（途经点/终点/规划线/行驶轨迹全清，艇停）
  - "2D Nav Goal" 点一下              → 设定/替换终点，**立即**以艇当前位置为起点规划并发布
                                        （艇是否启动由自动模式开关/遥控器决定）
  - 终端 commit（保留兼容）            → 只用已收集的途经点提交（无终点）
  - clear                             → 同"图外点击"：全部清空

规划完成后途经点与终点标记自动清空，下一次从新序列开始。
起点永远是艇当前位置（TF map→body，由 a_star_planner 查询）。

用法:
  1. 起 A*（真艇 a_star_demo.launch / 岸上练习 a_star_demo_sim.launch）
  2. rosrun a_star_planner waypoint_collector.py
  3. RViz："Publish Point" 点途经点 → "2D Nav Goal" 点终点 → 自动规划
"""
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

LINE_ID = 999
GOAL_ID = 2000
GOAL_TEXT_ID = 2100


class WaypointCollector(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'map')
        self.topic = rospy.get_param('~publish_topic', '/goal_sequence')
        self.auto_commit_sec = float(rospy.get_param('~auto_commit_sec', 0.0))
        self.auto_clear_out_of_map = bool(
            rospy.get_param('~auto_clear_out_of_map', True))

        self.points = []            # 途经点 [(x, y), ...]
        self.goal = None            # 终点 (x, y) 或 None
        self.last_click = None
        self.map_grid = None        # OccupancyGrid，用于图内/图外判别
        self.map_warned = False

        self.goal_pub = rospy.Publisher(self.topic, Path, queue_size=1)
        # clear 时向任务路径话题发一条空 Path：huamo 会停车并清空轨迹，RViz 绿线一并消失
        self.path_topic = rospy.get_param('~path_topic', '/waypoints_path')
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        # 标记用 latched：RViz 晚启动/重启也能立刻看到已收集的航点
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray,
                                          queue_size=1, latch=True)

        rospy.Subscriber('/clicked_point', PointStamped, self.on_click, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped,
                         self.on_nav_goal, queue_size=1)
        rospy.Subscriber('/prior_map', OccupancyGrid, self.on_map, queue_size=1)
        rospy.Subscriber('~commit', Empty, self.on_commit, queue_size=1)
        rospy.Subscriber('~clear', Empty, self.on_clear, queue_size=1)

        if self.auto_commit_sec > 0:
            rospy.Timer(rospy.Duration(1.0), self.tick_auto_commit)

        rospy.loginfo('waypoint_collector v2 就绪')
        rospy.loginfo('  途经点 : RViz "Publish Point" 点水域=设途经点（点岸上/障碍/图外=清除整条轨迹）')
        rospy.loginfo('  终点   : RViz "2D Nav Goal" 点一下 → 立即从艇当前位置规划')
        rospy.loginfo('  再点一次 2D Nav Goal = 替换终点并重新规划')
        rospy.loginfo('  兼容   : commit=仅提交途经点 / clear=全部清空')

    # ---------- 地图 ----------
    def on_map(self, msg):
        self.map_grid = msg

    def _is_click_clear(self, x, y):
        """清除判别：点击位置在图外，或落在岸上/障碍/未知栅格（非自由水域）→ 清除。
        RViz 的 Publish Point 只对渲染几何生效，"图外空白处"点不到，
        而"点岸上=清除"一定点得到。无地图时放行（交由规划器判别）。"""
        g = self.map_grid
        if g is None:
            if not self.map_warned:
                rospy.logwarn('尚未收到 /prior_map，无法判别水域/陆地'
                              '（点非水域=清除的功能暂不生效）')
                self.map_warned = True
            return False
        ox, oy = g.info.origin.position.x, g.info.origin.position.y
        w = g.info.width * g.info.resolution
        h = g.info.height * g.info.resolution
        if not (ox <= x <= ox + w and oy <= y <= oy + h):
            return True                                   # 图外
        col = int((x - ox) / g.info.resolution)
        row = int((y - oy) / g.info.resolution)
        try:
            return g.data[row * g.info.width + col] != 0  # 非自由栅格（岸/障碍/未知）
        except IndexError:
            return True

    # ---------- 交互 ----------
    def on_click(self, msg):
        p = msg.point
        if self.auto_clear_out_of_map and self._is_click_clear(p.x, p.y):
            rospy.loginfo('点击位置 (%.2f, %.2f) 为陆地/障碍/图外 → 清除整条轨迹'
                          '（点水域=设途经点，点非水域=清除）', p.x, p.y)
            self.on_clear(None)
            return
        self.points.append((p.x, p.y))
        self.last_click = rospy.Time.now()
        rospy.loginfo('已收集第 %d 个途经点: (%.2f, %.2f)'
                      '（终点用 "2D Nav Goal" 设置）', len(self.points), p.x, p.y)
        self.publish_markers()

    def on_nav_goal(self, msg):
        """2D Nav Goal：设/替换终点并立即规划。"""
        if self.auto_clear_out_of_map and self._is_click_clear(msg.pose.position.x,
                                                              msg.pose.position.y):
            rospy.loginfo('终点位置 (%.2f, %.2f) 为陆地/障碍/图外 → 清除整条轨迹',
                          msg.pose.position.x, msg.pose.position.y)
            self.on_clear(None)
            return
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = rospy.Time.now()
        for x, y in self.points + [self.goal]:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.goal_pub.publish(path)
        n = len(self.points)
        rospy.loginfo('已设定终点 (%.2f, %.2f)，从艇当前位置规划: %d 个途经点 + 终点 → %s',
                      self.goal[0], self.goal[1], n, self.topic)
        # 规划后清空本轮序列（途经点/终点标记消失），下一次从新序列开始
        self.points = []
        self.goal = None
        self.last_click = None
        self.publish_markers()

    def on_commit(self, _msg):
        """兼容保留：只提交途经点（无终点）。"""
        if not self.points:
            rospy.logwarn('还没有点任何途经点，忽略提交')
            return
        path = Path()
        path.header.frame_id = self.frame
        path.header.stamp = rospy.Time.now()
        for x, y in self.points:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)
        self.goal_pub.publish(path)
        rospy.loginfo('已提交 %d 个途经点（无终点）→ %s', len(self.points), self.topic)
        self.points = []
        self.last_click = None
        self.publish_markers()

    def on_clear(self, _msg):
        n = len(self.points)
        had_goal = self.goal is not None
        self.points = []
        self.goal = None
        self.last_click = None
        self.publish_markers()
        # 一键全清：空 Path 发到任务路径话题
        #   -> huamo 取消任务并停车、清空 /motion_path、/LOS_path
        #   -> RViz 绿线清空
        empty = Path()
        empty.header.frame_id = self.frame
        empty.header.stamp = rospy.Time.now()
        self.path_pub.publish(empty)
        rospy.loginfo('已清空 %d 个途经点%s，并重置规划路径与轨迹'
                      '（若艇正在执行任务会立即停车）',
                      n, '和终点' if had_goal else '')

    def tick_auto_commit(self, _evt):
        if not self.points or self.last_click is None:
            return
        if (rospy.Time.now() - self.last_click).to_sec() >= self.auto_commit_sec:
            self.on_commit(None)

    # ---------- 可视化 ----------
    def publish_markers(self):
        arr = MarkerArray()
        now = rospy.Time.now()

        # 先删除旧标记
        dele = Marker()
        dele.header.frame_id = self.frame
        dele.header.stamp = now
        dele.action = Marker.DELETEALL
        arr.markers.append(dele)

        def sphere_marker(x, y, ns, mid, r, g, b, text=None):
            m = Marker()
            m.header.frame_id = self.frame
            m.header.stamp = now
            m.ns = ns
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.85
            arr.markers.append(m)
            if text:
                t = Marker()
                t.header = m.header
                t.ns = ns + '_text'
                t.id = mid + 10000
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = x
                t.pose.position.y = y
                t.pose.position.z = 1.2
                t.pose.orientation.w = 1.0
                t.scale.z = 0.8
                t.color.r = t.color.g = t.color.b = 1.0
                t.color.a = 1.0
                t.text = text
                arr.markers.append(t)

        if self.points:
            line = Marker()
            line.header.frame_id = self.frame
            line.header.stamp = now
            line.ns = 'wp_line'
            line.id = LINE_ID
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.15
            line.color.r = 1.0
            line.color.g = 0.8
            line.color.a = 0.9
            line.pose.orientation.w = 1.0
            for i, (x, y) in enumerate(self.points):
                pt = Point()
                pt.x = x
                pt.y = y
                pt.z = 0.0
                line.points.append(pt)
                sphere_marker(x, y, 'wp', i, 0.2, 0.8, 1.0, str(i + 1))
            arr.markers.append(line)

        if self.goal:
            sphere_marker(self.goal[0], self.goal[1], 'goal', GOAL_ID,
                          1.0, 0.2, 0.2, '终点')

        self.marker_pub.publish(arr)


if __name__ == '__main__':
    rospy.init_node('waypoint_collector')
    WaypointCollector()
    rospy.spin()

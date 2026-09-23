#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RViz 点击收集多目标航点 → 一键提交给 A* 串规划。

简化操作流程：不再需要"点一个点 → 读坐标 → 手打指令"。
RViz 里连续点目标（带编号标记实时可见），点完一条命令提交。

用法:
  1. 起 A*（真艇 a_star_demo.launch / 岸上练习 a_star_demo_sim.launch）
  2. rosrun a_star_planner waypoint_collector.py
  3. RViz 用 "Publish Point" 工具逐个点目标
  4. 提交:  rostopic pub -1 /waypoint_collector/commit std_msgs/Empty
     清空:  rostopic pub -1 /waypoint_collector/clear  std_msgs/Empty

参数:
  ~frame            目标点/标记坐标系, 默认 map
  ~publish_topic    目标序列发布话题, 默认 /goal_sequence
  ~auto_commit_sec  最后一次点击后 N 秒自动提交; 0=关闭(默认, 显式提交更安全)
"""
import rospy
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from visualization_msgs.msg import Marker, MarkerArray

LINE_ID = 999


class WaypointCollector(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'map')
        self.topic = rospy.get_param('~publish_topic', '/goal_sequence')
        self.auto_commit_sec = float(rospy.get_param('~auto_commit_sec', 0.0))

        self.points = []
        self.last_click = None

        self.goal_pub = rospy.Publisher(self.topic, Path, queue_size=1)
        # clear 时向任务路径话题发一条空 Path：huamo 会停车并清空轨迹，RViz 绿线一并消失
        self.path_topic = rospy.get_param('~path_topic', '/waypoints_path')
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=1)
        # 标记用 latched：RViz 晚启动/重启也能立刻看到已收集的航点
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray,
                                          queue_size=1, latch=True)

        rospy.Subscriber('/clicked_point', PointStamped, self.on_click, queue_size=10)
        rospy.Subscriber('~commit', Empty, self.on_commit, queue_size=1)
        rospy.Subscriber('~clear', Empty, self.on_clear, queue_size=1)

        if self.auto_commit_sec > 0:
            rospy.Timer(rospy.Duration(1.0), self.tick_auto_commit)

        rospy.loginfo('waypoint_collector 就绪')
        rospy.loginfo('  点目标 : RViz "Publish Point" 工具')
        rospy.loginfo('  提交   : rostopic pub -1 %s std_msgs/Empty',
                      rospy.resolve_name('~commit'))
        rospy.loginfo('  清空   : rostopic pub -1 %s std_msgs/Empty',
                      rospy.resolve_name('~clear'))
        rospy.loginfo('  提交后发布到: %s', self.topic)

    # ---------- 交互 ----------
    def on_click(self, msg):
        p = msg.point
        self.points.append((p.x, p.y))
        self.last_click = rospy.Time.now()
        rospy.loginfo('已收集第 %d 个航点: (%.2f, %.2f)',
                      len(self.points), p.x, p.y)
        self.publish_markers()

    def on_commit(self, _msg):
        if not self.points:
            rospy.logwarn('还没有点任何航点，忽略提交')
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
        rospy.loginfo('已提交 %d 个航点 → %s（如需重发可再次 commit；'
                      '如需清空用 clear）', len(self.points), self.topic)

    def on_clear(self, _msg):
        n = len(self.points)
        self.points = []
        self.last_click = None
        self.publish_markers()
        # 一键全清：空 Path 发到任务路径话题
        #   -> huamo 取消任务并停车、清空 /motion_path、/LOS_path
        #   -> RViz 绿线清空
        empty = Path()
        empty.header.frame_id = self.frame
        empty.header.stamp = rospy.Time.now()
        self.path_pub.publish(empty)
        rospy.loginfo('已清空 %d 个航点，并重置规划路径与轨迹'
                      '（若艇正在执行任务会立即停车）', n)

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

                sphere = Marker()
                sphere.header = line.header
                sphere.ns = 'wp'
                sphere.id = i
                sphere.type = Marker.SPHERE
                sphere.action = Marker.ADD
                sphere.pose.position.x = x
                sphere.pose.position.y = y
                sphere.pose.orientation.w = 1.0
                sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.6
                sphere.color.b = 1.0
                sphere.color.a = 0.85
                arr.markers.append(sphere)

                text = Marker()
                text.header = line.header
                text.ns = 'wp_text'
                text.id = 100 + i
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD
                text.pose.position.x = x
                text.pose.position.y = y
                text.pose.position.z = 1.2
                text.pose.orientation.w = 1.0
                text.scale.z = 0.8
                text.color.r = text.color.g = text.color.b = 1.0
                text.color.a = 1.0
                text.text = str(i + 1)
                arr.markers.append(text)

            arr.markers.append(line)

        self.marker_pub.publish(arr)


if __name__ == '__main__':
    rospy.init_node('waypoint_collector')
    WaypointCollector()
    rospy.spin()

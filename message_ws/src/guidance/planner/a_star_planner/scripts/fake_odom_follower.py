#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""虚拟艇（陆地仿真演示专用）：订阅 /waypoints_path，沿路径匀速运动，
发布 /Odometry（camera_init 系）和 TF（camera_init -> base_link）。

纯运动学重放，不依赖任何传感器，只读路径不发布任何推进指令。
配合 a_star_demo_sim.launch 使用：huamo 消费 /Odometry 真实计算 LOS 与
PWM 输出（该 launch 不启动 usart/thruster_manager，PWM 不会到达硬件）。
"""
import math

import rospy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import TransformStamped


class FakeFollower(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'camera_init')
        self.speed = float(rospy.get_param('~speed', 0.5))
        self.rate_hz = float(rospy.get_param('~rate', 20.0))
        self.path = None      # [(x, y), ...]
        self.cum = []         # 累计弧长
        self.arc = 0.0        # 已走弧长
        self.last_yaw = 0.0
        self.last_t = rospy.get_rostime()

        rospy.Subscriber('/waypoints_path', Path, self.on_path, queue_size=2)
        self.odom_pub = rospy.Publisher('/Odometry', Odometry, queue_size=20)
        self.tf_br = tf2_ros.TransformBroadcaster()
        rospy.loginfo('fake_odom_follower: speed=%.2f m/s, frame=%s',
                      self.speed, self.frame)

    def on_path(self, msg):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(pts) < 2:
            return
        self.path = pts
        self.cum = [0.0]
        for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
        self.arc = 0.0
        self.last_t = rospy.get_rostime()
        rospy.loginfo('fake_odom_follower: 新路径 %d 点, 总长 %.1f m, 以 %.1f m/s 重放',
                      len(pts), self.cum[-1], self.speed)

    def pose_at(self, s):
        cum, pts = self.cum, self.path
        i = 1
        n = len(cum)
        while i < n and cum[i] < s:
            i += 1
        if i >= n:
            x, y = pts[-1]
            return x, y, self.last_yaw
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        seg = cum[i] - cum[i - 1]
        t = 0.0 if seg <= 1e-9 else (s - cum[i - 1]) / seg
        self.last_yaw = math.atan2(y1 - y0, x1 - x0)
        return x0 + t * (x1 - x0), y0 + t * (y1 - y0), self.last_yaw

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.get_rostime()
            dt = (now - self.last_t).to_sec()
            self.last_t = now
            dt = min(max(dt, 0.0), 0.2)  # 阻塞后防跳变

            if self.path and self.arc < self.cum[-1]:
                self.arc = min(self.arc + self.speed * dt, self.cum[-1])
                x, y, yaw = self.pose_at(self.arc)

                odom = Odometry()
                odom.header.stamp = now
                odom.header.frame_id = self.frame
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = x
                odom.pose.pose.position.y = y
                odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
                odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
                odom.twist.twist.linear.x = self.speed
                self.odom_pub.publish(odom)

                tf = TransformStamped()
                tf.header.stamp = now
                tf.header.frame_id = self.frame
                tf.child_frame_id = 'base_link'
                tf.transform.translation.x = x
                tf.transform.translation.y = y
                tf.transform.rotation = odom.pose.pose.orientation
                self.tf_br.sendTransform(tf)

            rate.sleep()


if __name__ == '__main__':
    rospy.init_node('fake_odom_follower')
    FakeFollower().spin()

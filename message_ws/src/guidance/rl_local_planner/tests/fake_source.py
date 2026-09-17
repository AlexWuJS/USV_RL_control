#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""节点实跑验证的假数据源【在小艇 NX 执行，仅测试用】：
- /waypoints_path：map 系沿 +x 直线 50m，只发前 2 秒
- /Odometry：camera_init 系，0.2 m/s 沿 +x 移动，10Hz
"""
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

rospy.init_node("fake_source")
odom_pub = rospy.Publisher("/Odometry", Odometry, queue_size=1)
path_pub = rospy.Publisher("/waypoints_path", Path, queue_size=1)

path = Path()
path.header.frame_id = "map"
for i in range(101):
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = i * 0.5
    ps.pose.orientation.w = 1.0
    path.poses.append(ps)

x, cycle = 0.0, 0
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    cycle += 1
    x += 0.02
    m = Odometry()
    m.header.stamp = rospy.Time.now()
    m.header.frame_id = "camera_init"
    m.pose.pose.position.x = x
    m.pose.pose.orientation.w = 1.0
    odom_pub.publish(m)
    if cycle <= 20:                      # 前 2 秒发路径
        path.header.stamp = rospy.Time.now()
        path_pub.publish(path)
    rate.sleep()

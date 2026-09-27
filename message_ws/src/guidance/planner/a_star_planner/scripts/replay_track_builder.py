#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""bag 回放辅助：把回放的 /Odometry 累积成一条 Path（/replay_track），
用于在 RViz 里显示艇走过的轨迹（bag 里没录 /motion_path）。"""
import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

rospy.init_node('replay_track_builder')
pub = rospy.Publisher('/replay_track', Path, queue_size=1, latch=True)
track = Path()
track.header.frame_id = 'camera_init'

def cb(m):
    ps = PoseStamped()
    ps.header = m.header
    ps.pose = m.pose.pose
    track.poses.append(ps)
    track.header.stamp = m.header.stamp
    pub.publish(track)

rospy.Subscriber('/Odometry', Odometry, cb)
rospy.spin()

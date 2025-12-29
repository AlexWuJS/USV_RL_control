#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import

import copy
import _thread
import time

import numpy as np
import rospy
import tf
import tf.transformations
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry

cur_odom_to_baselink = None  # 当前里程计到基座的变换矩阵
cur_map_to_odom = None       # 当前地图到里程计的变换矩阵
last_tf_timestamp = None     # 上次发布TF的时间戳，避免重复发布


def pose_to_mat(pose_msg):      # 将位姿消息转换为变换矩阵
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),
    )


def transform_fusion():     # 变换融合
    global cur_odom_to_baselink, cur_map_to_odom, last_tf_timestamp

    br = tf.TransformBroadcaster()    # 创建一个变换广播器
    while True:
        time.sleep(1 / FREQ_PUB_LOCALIZATION)    # 等待1/FREQ_PUB_LOCALIZATION秒

        # TODO 这里注意线程安全
        cur_odom = copy.copy(cur_odom_to_baselink)  # 复制当前里程计到基座的变换矩阵
        if cur_map_to_odom is not None:
            T_map_to_odom = pose_to_mat(cur_map_to_odom)  # 将当前地图到里程计的变换矩阵转换为变换矩阵
        else:
            T_map_to_odom = np.eye(4)  # 如果当前地图到里程计的变换矩阵为None，则设置为单位矩阵

        # 使用里程计的时间戳，避免在bag回放模式下时间戳重复
        if cur_odom is not None:
            tf_timestamp = cur_odom.header.stamp
        else:
            tf_timestamp = rospy.Time.now()

        # 只在时间戳变化时才发布TF，避免重复发布相同时间戳的数据
        if last_tf_timestamp is None or tf_timestamp != last_tf_timestamp:
            br.sendTransform(tf.transformations.translation_from_matrix(T_map_to_odom),
                             tf.transformations.quaternion_from_matrix(T_map_to_odom),
                             tf_timestamp,
                             'camera_init', 'map')  # 发布当前地图到里程计的变换矩阵到/map_to_odom主题        计算并发布map到camera_init的变换
            last_tf_timestamp = tf_timestamp
        if cur_odom is not None:
            # 发布全局定位的odometry   如果有里程计数据，计算并发布全局定位信息
            localization = Odometry()
            T_odom_to_base_link = pose_to_mat(cur_odom)  # 将当前里程计到基座的变换矩阵转换为变换矩阵
            # 这里T_map_to_odom短时间内变化缓慢 暂时不考虑与T_odom_to_base_link时间同步
            T_map_to_base_link = np.matmul(T_map_to_odom, T_odom_to_base_link)  # 将当前地图到基座的变换矩阵转换为变换矩阵
            xyz = tf.transformations.translation_from_matrix(T_map_to_base_link)  # 将当前地图到基座的变换矩阵转换为平移向量        
            quat = tf.transformations.quaternion_from_matrix(T_map_to_base_link)  # 将当前地图到基座的变换矩阵转换为四元数
            localization.pose.pose = Pose(Point(*xyz), Quaternion(*quat))  # 设置当前地图到基座的变换矩阵
            localization.twist = cur_odom.twist  # 设置当前里程计的twist

            localization.header.stamp = cur_odom.header.stamp  # 设置当前里程计的时间戳
            localization.header.frame_id = 'map'  # 设置当前地图的frame_id
            localization.child_frame_id = 'body'  # 设置当前基座的frame_id
            # rospy.loginfo_throttle(1, '{}'.format(np.matmul(T_map_to_odom, T_odom_to_base_link)))
            pub_localization.publish(localization)  # 发布当前地图到基座的变换矩阵


def cb_save_cur_odom(odom_msg):    # 保存当前里程计到基座的变换矩阵
    global cur_odom_to_baselink    # 全局变量，用于保存当前里程计到基座的变换矩阵
    cur_odom_to_baselink = odom_msg  # 更新当前里程计到基座的变换矩阵


def cb_save_map_to_odom(odom_msg):    # 保存当前地图到里程计的变换矩阵
    global cur_map_to_odom  # 全局变量，用于保存当前地图到里程计的变换矩阵
    cur_map_to_odom = odom_msg  # 更新当前地图到里程计的变换矩阵    


if __name__ == '__main__':
    # tf and localization publishing frequency (HZ)
    FREQ_PUB_LOCALIZATION = 50  # 发布全局定位的频率

    rospy.init_node('transform_fusion')  # 初始化节点
    rospy.loginfo('Transform Fusion Node Inited...')  # 日志信息

    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1)  # 订阅里程计消息
    rospy.Subscriber('/map_to_odom', Odometry, cb_save_map_to_odom, queue_size=1)  # 订阅地图到里程计的消息

    pub_localization = rospy.Publisher('/localization', Odometry, queue_size=1)  # 发布全局定位消息

    # 启动线程发布全局定位消息
    _thread.start_new_thread(transform_fusion, ())

    rospy.spin()

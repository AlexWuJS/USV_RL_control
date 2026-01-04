#!/usr/bin/python3
# coding=utf8
from __future__ import print_function, division, absolute_import # 导入future模块   

import copy # 导入copy模块
import _thread # 导入_thread模块
import time # 导入time模块

import open3d as o3d # 导入open3d模块
import rospy # 导入rospy模块
import ros_numpy # 导入ros_numpy模块
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion # 导入geometry_msgs.msg模块中的PoseWithCovarianceStamped, Pose, Point, Quaternion消息类型
from nav_msgs.msg import Odometry # 导入nav_msgs.msg模块中的Odometry消息类型
from sensor_msgs.msg import PointCloud2 # 导入sensor_msgs.msg模块中的PointCloud2消息类型
import numpy as np # 导入numpy模块
import tf # 导入tf模块
import tf.transformations # 导入tf.transformations模块

# 全局变量定义
global_map = None        # 全局地图点云
initialized = False      # 初始化状态标志
T_map_to_odom = np.eye(4)  # 地图到里程计的变换矩阵
cur_odom = None         # 当前里程计数据
cur_scan = None         # 当前激光扫描数据

# 添加发布者
pub_relocalization_success = None  # 重定位成功发布者

def pose_to_mat(pose_msg):
    #将ROS的pose消息转换为4x4变换矩阵
    return np.matmul(
        tf.listener.xyz_to_mat44(pose_msg.pose.pose.position),      # 位置转换矩阵
        tf.listener.xyzw_to_mat44(pose_msg.pose.pose.orientation),  # 旋转转换矩阵
    )


def msg_to_array(pc_msg):
    # 将ROS的PointCloud2消息转换为numpy数组
    pc_array = ros_numpy.numpify(pc_msg)  # 转换为numpy数组
    pc = np.zeros([len(pc_array), 3])     # 创建点云数组 形状为(N,3)的numpy数组，其中N是点的数量，3表示(x,y,z)坐标
    pc[:, 0] = pc_array['x']  # 将所有点的x坐标存入第0列
    pc[:, 1] = pc_array['y']  # 将所有点的y坐标存入第1列
    pc[:, 2] = pc_array['z']  # 提取z坐标
    return pc


def registration_at_scale(pc_scan, pc_map, initial, scale): 
    #pc_scan: 当前激光雷达扫描的点云 pc_map: 参考地图点云 initial: 初始位姿估计（4x4变换矩阵） scale: 配准的尺度因子（用于粗配准和精配准）
    #transformation: 配准后的变换矩阵 fitness: 配准的拟合度分数
    # 使用ICP算法进行点云配准
    result_icp = o3d.pipelines.registration.registration_icp(
        voxel_down_sample(pc_scan, SCAN_VOXEL_SIZE * scale), voxel_down_sample(pc_map, MAP_VOXEL_SIZE * scale),  # 对当前扫描点云降采样 
        1.0 * scale, initial,   # 设置缩放因子  对应点最大距离阈值 距离阈值决定配准时寻找对应点的范围
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),  # 使用点对点变换估计  直接最小化对应点之间的欧氏距离
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=20) # 设置ICP算法的最大迭代次数 当达到最大迭代次数或误差变化小于阈值时停止
    ) 

    return result_icp.transformation, result_icp.fitness # 返回变换矩阵和拟合度


def inverse_se3(trans):
    # 计算SE3变换矩阵的逆
    trans_inverse = np.eye(4)
    # 旋转矩阵的逆是其转置
    trans_inverse[:3, :3] = trans[:3, :3].T
    # 平移向量的逆需要通过旋转矩阵的转置进行变换
    trans_inverse[:3, 3] = -np.matmul(trans[:3, :3].T, trans[:3, 3])      
    return trans_inverse


def publish_point_cloud(publisher, header, pc):
    # 用于将处理后的点云数据发布到ROS系统中，便于其他节点订阅和使用
    # 参数：publisher: ROS发布者对象  header: ROS消息头 pc: 点云数据，numpy数组格式
    data = np.zeros(len(pc), dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('intensity', np.float32),
    ])
    data['x'] = pc[:, 0]
    data['y'] = pc[:, 1]
    data['z'] = pc[:, 2]
    if pc.shape[1] == 4:
        data['intensity'] = pc[:, 3]
    msg = ros_numpy.msgify(PointCloud2, data)
    msg.header = header
    publisher.publish(msg)

#提取当前激光雷达视野范围内的地图点云
def crop_global_map_in_FOV(global_map, pose_estimation, cur_odom):
    # # 参数：
    #     global_map: 全局地图点云
    #     pose_estimation: 当前位姿估计
    #     cur_odom: 当前里程计数据
    # 返回：
    #     global_map_in_FOV: 视野范围内的地图点云
    # 1. 计算当前扫描原点的位姿
    T_odom_to_base_link = pose_to_mat(cur_odom) # 将当前里程计数据转换为4x4变换矩阵
    T_map_to_base_link = np.matmul(pose_estimation, T_odom_to_base_link) # 将当前位姿估计和里程计数据转换为4x4变换矩阵 得到地图到基座的变换
    T_base_link_to_map = inverse_se3(T_map_to_base_link) # 计算逆变换矩阵

    # # 2. 将地图点云转换到激光雷达坐标系
    global_map_in_map = np.array(global_map.points)
    global_map_in_map = np.column_stack([global_map_in_map, np.ones(len(global_map_in_map))])
    global_map_in_base_link = np.matmul(T_base_link_to_map, global_map_in_map.T).T

    # 将视角内的地图点提取出来
    if FOV > 3.14:
        # 环状lidar 仅过滤距离
        indices = np.where(
            (global_map_in_base_link[:, 0] < FOV_FAR) &
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    else:
        # 非环状lidar 保前视范围
        # FOV_FAR>x>0 且角度小于FOV  FOV_FAR：最大视距，决定激光雷达的最远探测距离
        indices = np.where(
            (global_map_in_base_link[:, 0] > 0) &
            (global_map_in_base_link[:, 0] < FOV_FAR) & 
            (np.abs(np.arctan2(global_map_in_base_link[:, 1], global_map_in_base_link[:, 0])) < FOV / 2.0)
        )
    global_map_in_FOV = o3d.geometry.PointCloud() #创建一个新的点云对象，存储视野内的点。
    global_map_in_FOV.points = o3d.utility.Vector3dVector(np.squeeze(global_map_in_map[indices, :3])) # 将视野内的点存储到点云对象中    

    # 发布fov内点云
    header = cur_odom.header
    header.frame_id = 'map'
    publish_point_cloud(pub_submap, header, np.array(global_map_in_FOV.points)[::10])

    return global_map_in_FOV


def global_localization(pose_estimation):
    global global_map, cur_scan, cur_odom, T_map_to_odom
    # 用icp配准
    # print(global_map, cur_scan, T_map_to_odom)
    rospy.loginfo('Global localization by scan-to-map matching......')

    # TODO 这里注意线程安全 # 复制当前扫描数据，注意线程安全
    scan_tobe_mapped = copy.copy(cur_scan)

    tic = time.time()

    global_map_in_FOV = crop_global_map_in_FOV(global_map, pose_estimation, cur_odom) # 提取当前视野范围内的地图点云

    # 粗配准
    transformation, _ = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=pose_estimation, scale=5)

    # 精配准
    transformation, fitness = registration_at_scale(scan_tobe_mapped, global_map_in_FOV, initial=transformation,
                                                    scale=1)
    toc = time.time()
    rospy.loginfo('Time: {}'.format(toc - tic))
    rospy.logwarn('fitness score:{}'.format(fitness)) # 打印拟合度分数
    rospy.loginfo('')

    # 当全局定位成功时才更新map2odom
    if fitness > LOCALIZATION_TH:
        # 对新的定位结果进行平滑处理，避免突变导致地图抖动
        # 使用指数移动平均（EMA）平滑变换矩阵
        new_T_map_to_odom = transformation
        
        if T_map_to_odom is not None:
            # 计算位置差异
            pos_diff = np.linalg.norm(new_T_map_to_odom[:3, 3] - T_map_to_odom[:3, 3])
            
            # 自适应平滑：变化越大，平滑越强，但始终接受变化
            # 使用连续函数而不是阶跃函数，让过渡更自然
            if pos_diff > 3.0:
                # 超过3米的超大变化（机器人快速运动或重定位跳变）
                alpha = 0.08  # 极度平滑，需要更多次才完全收敛
                rospy.logwarn(f"Very large position change: {pos_diff:.3f}m, applying very strong smoothing (alpha={alpha})")
            elif pos_diff > 1.5:
                # 1.5-3米的大变化
                alpha = 0.12
                rospy.logwarn(f"Large position change: {pos_diff:.3f}m, applying strong smoothing (alpha={alpha})")
            elif pos_diff > 0.5:
                # 0.5-1.5米的中等变化
                alpha = 0.15
                rospy.loginfo(f"Medium position change: {pos_diff:.3f}m, applying medium smoothing (alpha={alpha})")
            elif pos_diff > 0.2:
                # 20-50cm的小变化
                alpha = 0.25
            else:
                # <20cm的微小变化，正常跟踪
                alpha = 0.35  # 进一步降低，更平滑
            
            # 位置平滑
            T_map_to_odom[:3, 3] = alpha * new_T_map_to_odom[:3, 3] + (1 - alpha) * T_map_to_odom[:3, 3]
            
            # 旋转平滑（使用四元数插值）
            old_quat = tf.transformations.quaternion_from_matrix(T_map_to_odom)
            new_quat = tf.transformations.quaternion_from_matrix(new_T_map_to_odom)
            # SLERP插值 - 球面线性插值保证旋转的平滑性
            smooth_quat = tf.transformations.quaternion_slerp(old_quat, new_quat, alpha)
            smooth_rot = tf.transformations.quaternion_matrix(smooth_quat)
            T_map_to_odom[:3, :3] = smooth_rot[:3, :3]
        else:
            # 第一次定位，直接使用新结果
            T_map_to_odom = new_T_map_to_odom
            rospy.loginfo("First localization, using new transformation directly")

        # 发布map_to_odom
        map_to_odom = Odometry()
        xyz = tf.transformations.translation_from_matrix(T_map_to_odom) # 从变换矩阵中提取平移向量  
        quat = tf.transformations.quaternion_from_matrix(T_map_to_odom) # 从变换矩阵中提取旋转四元数
        map_to_odom.pose.pose = Pose(Point(*xyz), Quaternion(*quat)) # 将平移向量和旋转四元数转换为Pose消息
        map_to_odom.header.stamp = cur_odom.header.stamp # 设置时间戳   
        map_to_odom.header.frame_id = 'map' # 设置坐标系
        pub_map_to_odom.publish(map_to_odom) # 发布map_to_odom消息
        
        # 发布重定位成功消息
        reloc_success = Odometry()
        reloc_success.header.stamp = rospy.Time.now()
        reloc_success.header.frame_id = 'map'
        reloc_success.pose.pose = map_to_odom.pose.pose
        pub_relocalization_success.publish(reloc_success)
        rospy.loginfo("Relocalization success published")
        
        return True
    else:
        rospy.logwarn('Not match!!!!') # 如果配准失败，打印警告信息
        #rospy.logwarn('{}'.format(transformation)) # 打印变换矩阵
        rospy.logwarn('fitness score:{}'.format(fitness)) # 打印拟合度分数
        return False # 返回False表示配准失败

# 对点云进行体素降采样，以减少点的数量，提高处理效率。
# 参数： pcd：输入的点云对象。 voxel_size：体素的大小，决定降采样的精度。
def voxel_down_sample(pcd, voxel_size):
    if voxel_size <= 0:
        return pcd
    try:
        pcd_down = pcd.voxel_down_sample(voxel_size) # Open3D新版本
    except Exception as e:
        # for opend3d 0.7 or lower
        # 这里其实可以直接raise，不建议用o3d.geometry.voxel_down_sample
        raise e
    return pcd_down

# 初始化全局地图点云
def initialize_global_map(pc_msg):
    global global_map # 存储降采样后的全局地图点云

    global_map = o3d.geometry.PointCloud() # 创建一个空的点云对象
    global_map.points = o3d.utility.Vector3dVector(msg_to_array(pc_msg)[:, :3]) # 将点云数据转换为点云对象
    global_map = voxel_down_sample(global_map, MAP_VOXEL_SIZE) # 对点云进行体素降采样
    rospy.loginfo('Global map received.') # 记录日志信息，表示全局地图已接收

# 保存当前里程计数据
def cb_save_cur_odom(odom_msg):
    global cur_odom
    cur_odom = odom_msg

# 保存当前激光雷达扫描数据
def cb_save_cur_scan(pc_msg):
    global cur_scan
    # 注意这里fastlio直接将scan转到odom系下了 不是lidar局部系
    pc_msg.header.frame_id = 'camera_init'
    pc_msg.header.stamp = rospy.Time().now()
    pub_pc_in_map.publish(pc_msg)

    # 转换为pcd
    # fastlio给的field有问题 处理一下
    pc_msg.fields = [pc_msg.fields[0], pc_msg.fields[1], pc_msg.fields[2],
                     pc_msg.fields[4], pc_msg.fields[5], pc_msg.fields[6],
                     pc_msg.fields[3], pc_msg.fields[7]]
    pc = msg_to_array(pc_msg)

    # 添加高度过滤
    if enable_height_filter:
        height_mask = (pc[:, 2] >= min_display_height) & (pc[:, 2] <= max_display_height)
        pc_filtered = pc[height_mask]
    else:
        pc_filtered = pc

    cur_scan = o3d.geometry.PointCloud()
    # 使用过滤后的点云
    cur_scan.points = o3d.utility.Vector3dVector(pc_filtered[:, :3])

# 线程局部化
# 每隔一段时间进行全局定位  
def thread_localization():
    global T_map_to_odom
    while True:
        # 每隔一段时间进行全局定位
        rospy.sleep(1 / FREQ_LOCALIZATION)
        # TODO 由于这里Fast lio发布的scan是已经转换到odom系下了 所以每次全局定位的初始解就是上一次的map2odom 不需要再拿odom了
        global_localization(T_map_to_odom)


if __name__ == '__main__':
    # 读取下采样参数 - 使用独立的参数名避免与map_loader冲突
    enable_downsample = rospy.get_param('enable_localization_downsample', True)
    if not enable_downsample:
        SCAN_VOXEL_SIZE = 0.0
        MAP_VOXEL_SIZE = 0.0
        rospy.loginfo('Localization downsample disabled')
    else:
        SCAN_VOXEL_SIZE = 0.1  # 默认值
        MAP_VOXEL_SIZE = 0.1  # 默认值
        rospy.loginfo(f'Localization downsample enabled: SCAN_VOXEL_SIZE={SCAN_VOXEL_SIZE}, MAP_VOXEL_SIZE={MAP_VOXEL_SIZE}')

    # 全局定位频率 (HZ)
    FREQ_LOCALIZATION = 0.3  # 每2秒一次，保持合理的定位更新频率

    # 全局定位的阈值，只有那些配准的拟合度高于LOCALIZATION_TH的扫描才会被使用
    LOCALIZATION_TH = 0.80  # 保持原始阈值，平衡定位成功率和准确性

    # FOV(弧度), 根据你的激光雷达类型修改
    FOV = 6.28

    # FOV内的最大距离(米)
    FOV_FAR = 30

    rospy.init_node('fast_lio_localization') # 初始化ROS节点
    rospy.loginfo('Localization Node Inited...') # 记录日志信息，表示节点已初始化

    # 从参数服务器读取高度范围参数
    global min_display_height, max_display_height
    min_display_height = rospy.get_param('min_display_height', -0.5)
    max_display_height = rospy.get_param('max_display_height', 1.0)
    # 新增：读取enable_height_filter参数
    global enable_height_filter
    enable_height_filter = rospy.get_param('enable_height_filter', True)
    rospy.loginfo(f'Height filter range: {min_display_height} to {max_display_height}, enable_height_filter: {enable_height_filter}')

    # publisher
    pub_pc_in_map = rospy.Publisher('/cur_scan_in_map', PointCloud2, queue_size=1)
    pub_submap = rospy.Publisher('/submap', PointCloud2, queue_size=1)
    pub_map_to_odom = rospy.Publisher('/map_to_odom', Odometry, queue_size=1)
    pub_relocalization_success = rospy.Publisher('/relocalization_success', Odometry, queue_size=1)  # 添加重定位成功发布者

    rospy.Subscriber('/cloud_registered', PointCloud2, cb_save_cur_scan, queue_size=1) # 订阅注册后的点云数据
    rospy.Subscriber('/Odometry', Odometry, cb_save_cur_odom, queue_size=1) # 订阅里程计数据

    # 初始化全局地图
    rospy.logwarn('Waiting for global map......')
    initialize_global_map(rospy.wait_for_message('/map', PointCloud2)) # 等待全局地图消息

    # 初始化
    while not initialized:
        rospy.logwarn('Waiting for initial pose....')

        # 等待初始位姿
        pose_msg = rospy.wait_for_message('/initialpose', PoseWithCovarianceStamped)
        initial_pose = pose_to_mat(pose_msg)
        if cur_scan:
            initialized = global_localization(initial_pose)
        else:
            rospy.logwarn('First scan not received!!!!!')

    rospy.loginfo('')
    rospy.loginfo('Initialize successfully!!!!!!')
    rospy.loginfo('')
    # 开始定期全局定位
    _thread.start_new_thread(thread_localization, ())

    rospy.spin()

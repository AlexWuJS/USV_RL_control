#!/usr/bin/env python
# coding=utf8
import rospy  # ROS Python库
import numpy as np  # 数值计算库
import open3d as o3d  # 点云处理库
from sensor_msgs.msg import PointCloud2  # ROS点云消息类型
import ros_numpy  # ROS和numpy数据转换库
import threading  # 线程库
import time  # 时间库
import os  # 操作系统接口库
from nav_msgs.msg import Odometry  # ROS里程计消息类型
import tf.transformations  # ROS坐标变换库
import tf2_ros  # ROS坐标变换库2
import geometry_msgs.msg  # ROS几何消息类型

class MapChunkLoader:
    """
    地图块加载器类
    主要功能：
    1. 加载点云地图文件
    2. 处理地图坐标系变换
    3. 发布变换后的点云地图
    4. 处理重定位事件
    """
    def __init__(self):
        """初始化函数，设置ROS节点和参数"""
        rospy.init_node('map_chunk_loader', anonymous=True)  # 初始化ROS节点
        
        # 从参数服务器获取配置参数
        self.pcd_path = rospy.get_param('~pcd_path', '')  # 点云文件路径
        self.voxel_size = rospy.get_param('~voxel_size', 0.2)  # 体素降采样大小
        self.display_ratio = rospy.get_param('~display_ratio', 1.0)  # 点云显示比例
        self.display_mode = rospy.get_param('~display_mode', 'random')  # 点云显示模式
        
        # 日志打印控制参数
        self.last_log_time = rospy.Time.now()  # 上次日志打印时间
        self.log_interval = rospy.Duration(10.0)  # 日志打印最小间隔
        self.last_transform_log_time = rospy.Time.now()  # 上次变换日志时间
        self.transform_log_interval = rospy.Duration(10.0)  # 变换日志最小间隔
        
        # 打印初始化参数
        rospy.loginfo(f"PCD path: {self.pcd_path}")
        rospy.loginfo(f"Parameters: voxel_size={self.voxel_size}, display_ratio={self.display_ratio}, display_mode={self.display_mode}")
        
        # 检查点云文件是否存在
        if not os.path.exists(self.pcd_path):
            rospy.logerr(f"PCD file not found: {self.pcd_path}")
            return
            
        # 创建点云发布者
        self.pub = rospy.Publisher('/map', PointCloud2, queue_size=1)
        
        # 初始化类成员变量
        self.full_map = None  # 原始点云地图
        self.transformed_map = None  # 变换后的点云地图
        self.T_map_to_odom = None  # 地图到里程计的变换矩阵
        self.initialized = False  # 初始化标志
        self.use_odom_frame = False  # 是否使用里程计坐标系
        self.last_transform_time = None  # 上次变换时间
        self.odom_frame_available = False  # 里程计坐标系是否可用
        self.last_publish_time = None  # 上次发布时间
        self.publish_interval = rospy.Duration(1.0)  # 发布间隔
        self.last_reloc_time = None  # 上次重定位时间
        self.reloc_interval = rospy.Duration(5.0)  # 重定位消息间隔
        
        # 创建TF监听器
        self.tf_buffer = tf2_ros.Buffer()  # TF缓冲区
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)  # TF监听器
        
        # 订阅重定位成功话题
        rospy.Subscriber('/relocalization_success', Odometry, self.relocalization_callback)
        
        # 加载地图
        self.load_map()
        
        # 创建定时器，定期检查并应用变换
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)
        
        # 打印初始化完成
        rospy.loginfo("Map chunk loader initialized")

    def check_odom_frame(self):
        """
        检查里程计坐标系(camera_init)是否可用
        返回值：
            bool: 坐标系是否可用
        """
        try:
            # 尝试获取camera_init坐标系到map坐标系的变换  rospy.Time(0)表示获取最新的可用变换 rospy.Duration(0.1)设置超时时间为0.1秒
            self.tf_buffer.lookup_transform('camera_init', 'map', rospy.Time(0), rospy.Duration(0.1)) 
            if not self.odom_frame_available: # 如果坐标系不可用 如果之前不可用，现在能获取到变换了 → 变成可用
                rospy.loginfo("camera_init frame is now available")
                self.odom_frame_available = True # 设置坐标系可用
            return True # 返回坐标系可用
        # 捕获异常  LookupException: 找不到变换 ConnectivityException: 坐标变换链断开 ExtrapolationException: 变换时间戳超出范围
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: 
            if self.odom_frame_available: # 如果坐标系可用  发生异常，返回False表示坐标系不可用
                rospy.logwarn(f"camera_init frame is temporarily unavailable: {str(e)}") # 打印警告
                self.odom_frame_available = False # 设置坐标系不可用
            return False # 返回坐标系不可用

    def relocalization_callback(self, msg):
        """
        重定位成功回调函数
        参数：
            msg: 重定位成功消息
        """
        current_time = rospy.Time.now()
        
        # 只在第一次收到消息时打印
        if self.last_reloc_time is None: # 如果上次重定位时间为None 表示第一次收到消息
            rospy.loginfo("Received relocalization success message for the first time")
            self.last_reloc_time = current_time # 更新上次重定位时间
            
            # 检查坐标系是否可用
            if self.check_odom_frame(): # 检查坐标系是否可用
                self.update_map_transform() # 更新地图变换矩阵  
            else:
                rospy.logwarn("Cannot update map transform: camera_init frame is not available") # 打印警告
        else:
            # 后续收到消息只更新地图，不打印消息
            if self.check_odom_frame(): # 检查坐标系是否可用
                self.update_map_transform() # 更新地图变换矩阵  
            else:
                rospy.logwarn("Cannot update map transform: camera_init frame is not available") # 打印警告

    def update_map_transform(self):
        """
        更新地图变换矩阵
        功能：
        1. 获取最新的坐标系变换
        2. 构建变换矩阵
        3. 应用变换到点云
        """
        if not self.odom_frame_available:#如果camera_init坐标系不可用，打印警告并直接返回
            rospy.logwarn("Cannot update transform: camera_init frame is not available")
            return
            
        try:
            # 获取最新的map到camera_init的变换 
            trans = self.tf_buffer.lookup_transform('camera_init', 'map', rospy.Time(0))
            
            # 构建变换矩阵
            T = np.eye(4)  # 创建4x4单位矩阵
            # 设置旋转部分
            T[:3, :3] = tf.transformations.quaternion_matrix([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ])[:3, :3]
            # 设置平移部分
            T[:3, 3] = [
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ]
            
            # 检查是否是第一次使用变换矩阵
            if self.T_map_to_odom is None: # 如果变换矩阵为None 表示第一次使用变换矩阵
                self.T_map_to_odom = T # 更新变换矩阵
                self.use_odom_frame = True # 设置使用里程计坐标系
                if self.full_map is not None: # 如果点云不为None
                    self.apply_transform() # 应用变换
                    rospy.loginfo("Map transformed and published in camera_init frame for the first time")
                return
            
            # 检查变换矩阵是否发生显著变化
            if not np.allclose(self.T_map_to_odom, T, rtol=1e-3, atol=1e-3):
                # 更新变换矩阵
                self.T_map_to_odom = T
                self.use_odom_frame = True
                
                # 应用变换
                if self.full_map is not None: # 如果点云不为None
                    self.apply_transform() # 应用变换
        # 捕获异常  LookupException: 找不到变换 ConnectivityException: 坐标变换链断开 ExtrapolationException: 变换时间戳超出范围
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"Failed to get transform: {str(e)}")
            self.odom_frame_available = False # 设置坐标系不可用
            # 只在发生错误且变换矩阵没有改变时打印消息
            if self.T_map_to_odom is not None:
                rospy.loginfo("Map transform failed but remains unchanged in camera_init frame")

    def timer_callback(self, event):
        """
        定时器回调函数
        功能：
        1. 定期检查坐标系变换
        2. 更新并发布点云
        """
        if not self.initialized: #如果类还没有初始化完成，直接返回
            return
            
        current_time = rospy.Time.now()
        # 如果上次发布时间为None 或者 当前时间减去上次发布时间大于发布间隔
        if self.last_publish_time is None or (current_time - self.last_publish_time) > self.publish_interval:
            # 检查坐标系是否可用
            if not self.check_odom_frame():
                # 如果坐标系不可用，继续使用map坐标系发布点云
                if self.full_map is not None: # 如果点云不为None
                    self.publish_map(self.full_map, "map", verbose=False) # 发布点云
                return # 返回
                
            try:
                # 获取最新的变换
                trans = self.tf_buffer.lookup_transform('camera_init', 'map', rospy.Time(0))
                
                # 检查变换是否发生变化  检查是否是第一次获取变换或变换时间戳发生变化
                if self.last_transform_time is None or trans.header.stamp != self.last_transform_time: 
                    self.last_transform_time = trans.header.stamp # 更新上次变换时间
                    self.update_map_transform() # 更新地图变换矩阵
                    
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e: # 捕获异常  LookupException: 找不到变换 ConnectivityException: 坐标变换链断开 ExtrapolationException: 变换时间戳超出范围
                rospy.logwarn(f"Failed to get transform: {str(e)}") # 打印警告
                self.odom_frame_available = False # 设置坐标系不可用

    def load_map(self):
        """
        加载点云地图
        功能：
        1. 读取PCD文件
        2. 根据显示比例和模式处理点云
        3. 进行体素降采样
        4. 初始化变换后的地图
        """
        try:
            # 加载PCD文件
            rospy.loginfo(f"Loading PCD file: {self.pcd_path}")
            pcd = o3d.io.read_point_cloud(self.pcd_path) # 读取PCD文件
            points = np.asarray(pcd.points) # 将点云转换为numpy数组
            total_points = len(points) # 计算点云总数
            rospy.loginfo(f"Total points in PCD: {total_points}") # 打印点云总数
            
            # 计算要显示的点数
            display_points = int(total_points * self.display_ratio) # 计算要显示的点数
            
            if self.display_mode == 'random': # 如果显示模式为随机
                # 随机采样显示整个地图的百分之多少
                indices = np.random.choice(total_points, display_points, replace=False) # 随机采样显示整个地图的百分之多少
                points = points[indices] # 将点云转换为numpy数组
                rospy.loginfo(f"Randomly sampling {display_points} points ({self.display_ratio*100}% of total)") # 打印随机采样点数
            else:
                # 显示前百分之多少的地图
                points = points[:display_points] # 显示前百分之多少的地图
                rospy.loginfo(f"Displaying first {display_points} points ({self.display_ratio*100}% of total)") # 打印显示点数
            
            # 创建点云对象
            self.full_map = o3d.geometry.PointCloud() # 创建点云对象
            self.full_map.points = o3d.utility.Vector3dVector(points) # 将点云转换为numpy数组
            
            # 降采样
            if self.voxel_size > 0: # 如果体素大小大于0
                self.full_map = self.full_map.voxel_down_sample(self.voxel_size) # 降采样
                rospy.loginfo(f"Points after voxel down sampling: {len(self.full_map.points)}") # 打印降采样后点云数量
            
            # 初始化变换后的地图
            self.transformed_map = o3d.geometry.PointCloud() # 创建点云对象
            self.transformed_map.points = self.full_map.points # 将点云转换为numpy数组
            
            self.initialized = True # 初始化完成
            rospy.loginfo("Map loaded successfully") # 打印加载成功
            
            # 初始发布到map坐标系
            self.publish_map(self.full_map, "map") # 发布点云
            
        except Exception as e:
            rospy.logerr(f"Error loading map: {str(e)}") # 打印加载失败

    def apply_transform(self):
        """
        应用变换到地图
        功能：
        1. 将变换矩阵应用到点云
        2. 更新变换后的点云
        3. 发布变换后的点云
        """
        if self.full_map is None or self.T_map_to_odom is None: # 如果点云为None或变换矩阵为None
            current_time = rospy.Time.now() # 获取当前时间
            if current_time - self.last_log_time > self.log_interval: # 如果当前时间减去上次日志时间大于日志间隔
                rospy.logwarn("Cannot apply transform: full_map or T_map_to_odom is None") # 打印警告
                self.last_log_time = current_time # 更新上次日志时间
            return
            
        try:
            # 获取原始点云
            points = np.asarray(self.full_map.points) # 将点云转换为numpy数组
            current_time = rospy.Time.now() # 获取当前时间
            
            # 只在点云数量发生变化时打印消息
            if not hasattr(self, 'last_point_count') or len(points) != self.last_point_count: # 如果点云数量发生变化
                if current_time - self.last_log_time > self.log_interval: # 如果当前时间减去上次日志时间大于日志间隔
                    rospy.loginfo(f"Applying transform to {len(points)} points") # 打印应用变换点数
                    self.last_log_time = current_time # 更新上次日志时间
                self.last_point_count = len(points) # 更新上次点云数量
            
            # 添加齐次坐标
            points_homogeneous = np.column_stack([points, np.ones(len(points))])
            # 应用变换
            points_transformed = np.dot(self.T_map_to_odom, points_homogeneous.T).T[:, :3]
            
            # 更新变换后的地图
            self.transformed_map = o3d.geometry.PointCloud()
            self.transformed_map.points = o3d.utility.Vector3dVector(points_transformed)
            
            # 只在点云发生变化时打印消息
            if not hasattr(self, 'last_transformed_points') or not np.array_equal(points_transformed, self.last_transformed_points):
                if current_time - self.last_log_time > self.log_interval:
                    rospy.loginfo("Transform applied to map")
                    self.last_log_time = current_time
                self.last_transformed_points = points_transformed
            
            # 发布变换后的地图
            self.publish_map(self.transformed_map, "camera_init")
            
        except Exception as e:
            current_time = rospy.Time.now()
            if current_time - self.last_log_time > self.log_interval:
                rospy.logerr(f"Error applying transform: {str(e)}")
                self.last_log_time = current_time

    def publish_map(self, pcd, frame_id, verbose=True):
        """
        发布点云地图
        参数：
            pcd: 要发布的点云
            frame_id: 坐标系ID
            verbose: 是否打印详细信息
        功能：
        1. 创建ROS消息
        2. 发布点云数据
        3. 记录发布时间
        """
        try:
            # 创建ROS消息头
            header = rospy.Header()  # 创建ROS消息头
            header.stamp = rospy.Time.now() # 获取当前时间
            header.frame_id = frame_id # 设置坐标系ID
            
            # 获取点云数据
            points = np.asarray(pcd.points) # 将点云转换为numpy数组
            
            # 创建PointCloud2消息
            data = np.zeros(len(points), dtype=[
                ('x', np.float32), 
                ('y', np.float32), 
                ('z', np.float32), 
            ])
            data['x'] = points[:, 0] # 设置x坐标
            data['y'] = points[:, 1] # 设置y坐标
            data['z'] = points[:, 2] # 设置z坐标
            
            # 转换为ROS消息
            msg = ros_numpy.msgify(PointCloud2, data) # 转换为ROS消息
            msg.header = header # 设置消息头
            
            # 发布消息
            self.pub.publish(msg) # 发布消息
            self.last_publish_time = rospy.Time.now() # 更新上次发布时间
            
            # 只在第一次发布或点云数量变化时打印消息
            if verbose: # 如果verbose为True
                current_time = rospy.Time.now() # 获取当前时间
                if not hasattr(self, 'last_published_points') or len(points) != self.last_published_points: # 如果点云数量发生变化
                    rospy.loginfo(f"Published map with {len(points)} points in {frame_id} frame") # 打印发布点云数量
                    self.last_published_points = len(points) # 更新上次发布点云数量
            
        except Exception as e:
            current_time = rospy.Time.now() # 获取当前时间  
            if current_time - self.last_log_time > self.log_interval: # 如果当前时间减去上次日志时间大于日志间隔
                rospy.logerr(f"Error publishing map: {str(e)}") # 打印错误
                self.last_log_time = current_time # 更新上次日志时间

if __name__ == '__main__':
    try:
        MapChunkLoader()  # 创建地图加载器实例
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        pass  # 处理ROS中断异常 
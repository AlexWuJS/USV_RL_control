// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver2/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>

#define INIT_TIME           (0.1)  // 初始化时间，单位为秒
#define LASER_POINT_COV     (0.001)  // 激光点的协方差
#define MAXN                (720000)  // 最大点数
#define PUBFRAME_PERIOD     (20)  // 发布帧的周期

/*** 时间日志变量 ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;  // KD树相关的时间变量
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];  // 用于记录时间的数组
double match_time = 0, solve_time = 0, solve_const_H_time = 0;  // 匹配和求解时间
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;  // KD树大小和计数器
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false;  // 运行时位置日志、PCD保存和时间同步的标志
/**************************/

float res_last[100000] = {0.0};  // 上一次的残差
float DET_RANGE = 300.0f;  // 检测范围
const float MOV_THRESHOLD = 1.5f;  // 移动阈值

mutex mtx_buffer;  // 互斥锁，用于线程同步
condition_variable sig_buffer;  // 条件变量，用于线程同步

string root_dir = ROOT_DIR;  // 根目录
string map_file_path, lid_topic, imu_topic;  // 地图文件路径、激光雷达和IMU话题

double min_display_height = -0.5;                   // 显示点云的最小高度---------------------------------------------------------//
double max_display_height = 1.0;                    // 显示点云的最大高度---------------------------------------------------------//
double res_mean_last = 0.05, total_residual = 0.0;  // 上一次的平均残差和总残差
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;  // 上一次激光雷达和IMU的时间戳
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;  // 陀螺仪和加速度计的协方差
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;  // 滤波器大小和视场角
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;  // 其他参数
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;  // 特征数量、时间日志计数器、扫描计数和发布计数
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0;  // 迭代计数、特征降采样大小、最大迭代次数和有效激光点数
bool   point_selected_surf[100000] = {0};  // 选中的表面点
bool   lidar_pushed, flg_reset, flg_exit = false, flg_EKF_inited;  // 激光雷达推送、重置、退出和EKF初始化的标志
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;  // 扫描发布、密集发布和扫描体发布的标志

vector<vector<int>>  pointSearchInd_surf;  // 表面点搜索索引
vector<BoxPointType> cub_needrm;  // 需要移除的立方体
vector<PointVector>  Nearest_Points;  // 最近点
vector<double>       extrinT(3, 0.0);  // 外参平移
vector<double>       extrinR(9, 0.0);  // 外参旋转
deque<double>                     time_buffer;  // 时间缓冲区
deque<PointCloudXYZI::Ptr>        lidar_buffer;  // 激光雷达缓冲区
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;  // IMU缓冲区

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());  // 从地图中提取的特征
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());  // 去畸变的特征
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  // 降采样后的特征（体坐标系）
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());  // 降采样后的特征（世界坐标系）
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));  // 法向量
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));  // 原始激光点云
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));  // 校正后的法向量
PointCloudXYZI::Ptr _featsArray;  // 特征数组

pcl::VoxelGrid<PointType> downSizeFilterSurf;  // 表面降采样滤波器
pcl::VoxelGrid<PointType> downSizeFilterMap;  // 地图降采样滤波器

KD_TREE ikdtree;  // KD树

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);  // 体坐标系下的X轴点
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);  // 世界坐标系下的X轴点
V3D euler_cur;  // 当前欧拉角
V3D position_last(Zero3d);  // 上一次的位置
V3D Lidar_T_wrt_IMU(Zero3d);  // 激光雷达相对于IMU的平移
M3D Lidar_R_wrt_IMU(Eye3d);  // 激光雷达相对于IMU的旋转

/*** EKF输入和输出 ***/
MeasureGroup Measures;  // 测量组
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;  // 扩展卡尔曼滤波器
state_ikfom state_point;  // 状态点
vect3 pos_lid;  // 激光雷达位置

nav_msgs::Path path;  // 路径
nav_msgs::Odometry odomAftMapped;  // 映射后的里程计
geometry_msgs::Quaternion geoQuat;  // 四元数
geometry_msgs::PoseStamped msg_body_pose;  // 体姿态消息

shared_ptr<Preprocess> p_pre(new Preprocess());  // 预处理对象
shared_ptr<ImuProcess> p_imu(new ImuProcess());  // IMU处理对象

void SigHandle(int sig)
{
    flg_exit = true;  // 设置退出标志为true
    ROS_WARN("catch sig %d", sig);  // 打印捕获的信号
    sig_buffer.notify_all();  // 通知所有等待的线程
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));  // 计算旋转矩阵的对数映射，得到旋转角
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);  // 打印激光雷达开始时间
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));  // 打印旋转角
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2));  // 打印位置
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // 打印角速度（此处为0）
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2));  // 打印速度
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);  // 打印加速度（此处为0）
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));  // 打印陀螺仪偏置
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));  // 打印加速度计偏置
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]);  // 打印重力
    fprintf(fp, "\r\n");  // 换行
    fflush(fp);  // 刷新文件缓冲区
}

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);  // 将输入点的坐标转换为V3D类型
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);  // 计算全局坐标

    po->x = p_global(0);  // 设置输出点的x坐标
    po->y = p_global(1);  // 设置输出点的y坐标
    po->z = p_global(2);  // 设置输出点的z坐标
    po->intensity = pi->intensity;  // 保留输入点的强度
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);  // 将输入点的坐标转换为V3D类型
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);  // 计算全局坐标

    po->x = p_global(0);  // 设置输出点的x坐标
    po->y = p_global(1);  // 设置输出点的y坐标
    po->z = p_global(2);  // 设置输出点的z坐标
    po->intensity = pi->intensity;  // 保留输入点的强度
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);  // 将输入点的坐标转换为V3D类型
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);  // 计算全局坐标

    po[0] = p_global(0);  // 设置输出点的x坐标
    po[1] = p_global(1);  // 设置输出点的y坐标
    po[2] = p_global(2);  // 设置输出点的z坐标
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);  // 将输入点的坐标转换为V3D类型
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);  // 计算全局坐标

    po->x = p_global(0);  // 设置输出点的x坐标
    po->y = p_global(1);  // 设置输出点的y坐标
    po->z = p_global(2);  // 设置输出点的z坐标
    po->intensity = pi->intensity;  // 保留输入点的强度
}

void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);  // 将输入点的坐标转换为V3D类型
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);  // 计算IMU坐标

    po->x = p_body_imu(0);  // 设置输出点的x坐标
    po->y = p_body_imu(1);  // 设置输出点的y坐标
    po->z = p_body_imu(2);  // 设置输出点的z坐标
    po->intensity = pi->intensity;  // 保留输入点的强度
}

void points_cache_collect()
{
    PointVector points_history;  // 创建一个点向量用于存储历史点
    ikdtree.acquire_removed_points(points_history);  // 从KD树中获取被移除的点
    for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);  // 将历史点添加到特征数组中
}

BoxPointType LocalMap_Points;  // 定义一个立方体类型的变量，用于存储局部地图的边界  
bool Localmap_Initialized = false;  // 标志变量，用于判断局部地图是否已经初始化
void lasermap_fov_segment()  // 激光雷达地图的FOV分割函数
{
    cub_needrm.clear();  // 清空需要移除的立方体列表
    kdtree_delete_counter = 0;  // KD树删除计数器置零
    kdtree_delete_time = 0.0;  // KD树删除时间置零
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);  // 将体坐标系下的X轴点转换到世界坐标系
    V3D pos_LiD = pos_lid;  // 获取激光雷达的位置
    if (!Localmap_Initialized){  // 如果局部地图未初始化
        for (int i = 0; i < 3; i++){  // 初始化局部地图的边界
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;  // 标记局部地图已初始化
        return;  // 退出函数
    }
    float dist_to_map_edge[3][2];  // 存储到地图边界的距离
    bool need_move = false;  // 标记是否需要移动地图
    for (int i = 0; i < 3; i++){  // 计算到地图边界的距离
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;  // 判断是否需要移动
    }
    if (!need_move) return;  // 如果不需要移动，退出函数
    BoxPointType New_LocalMap_Points, tmp_boxpoints;  // 定义新的局部地图边界和临时立方体
    New_LocalMap_Points = LocalMap_Points;  // 复制当前局部地图边界
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));  // 计算移动距离
    for (int i = 0; i < 3; i++){  // 更新局部地图边界
        tmp_boxpoints = LocalMap_Points;  // 复制当前局部地图边界
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){  // 如果需要向负方向移动
            New_LocalMap_Points.vertex_max[i] -= mov_dist;  // 更新最大边界
            New_LocalMap_Points.vertex_min[i] -= mov_dist;  // 更新最小边界
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;  // 更新临时立方体的最小边界
            cub_needrm.push_back(tmp_boxpoints);  // 将临时立方体添加到需要移除的列表
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){  // 如果需要向正方向移动
            New_LocalMap_Points.vertex_max[i] += mov_dist;  // 更新最大边界
            New_LocalMap_Points.vertex_min[i] += mov_dist;  // 更新最小边界
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;  // 更新临时立方体的最大边界
            cub_needrm.push_back(tmp_boxpoints);  // 将临时立方体添加到需要移除的列表
        }
    }
    LocalMap_Points = New_LocalMap_Points;  // 更新局部地图边界

    points_cache_collect();  // 收集点缓存
    double delete_begin = omp_get_wtime();  // 记录删除开始时间
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);  // 删除需要移除的立方体
    kdtree_delete_time = omp_get_wtime() - delete_begin;  // 计算删除时间
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();  // 锁定互斥量
    scan_count ++;  // 增加扫描计数
    double preprocess_start_time = omp_get_wtime();  // 记录预处理开始时间
    if (msg->header.stamp.toSec() < last_timestamp_lidar)  // 如果消息时间戳小于上次激光雷达时间戳
    {
        ROS_ERROR("lidar loop back, clear buffer");  // 打印错误信息
        lidar_buffer.clear();  // 清空激光雷达缓冲区
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());  // 创建新的点云指针
    p_pre->process(msg, ptr);  // 处理点云消息
    lidar_buffer.push_back(ptr);  // 将处理后的点云添加到缓冲区
    time_buffer.push_back(msg->header.stamp.toSec());  // 将时间戳添加到时间缓冲区
    last_timestamp_lidar = msg->header.stamp.toSec();  // 更新上次激光雷达时间戳
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  // 记录预处理时间
    mtx_buffer.unlock();  // 解锁互斥量
    sig_buffer.notify_all();  // 通知所有等待的线程
}

double timediff_lidar_wrt_imu = 0.0;  // 激光雷达与IMU的时间差
bool   timediff_set_flg = false;  // 时间差设置标志
void livox_pcl_cbk(const livox_ros_driver2::CustomMsg::ConstPtr &msg) 
{
    mtx_buffer.lock();  // 锁定互斥量
    double preprocess_start_time = omp_get_wtime();  // 记录预处理开始时间
    scan_count ++;  // 增加扫描计数
    if (msg->header.stamp.toSec() < last_timestamp_lidar)  // 如果消息时间戳小于上次激光雷达时间戳
    {
        ROS_ERROR("lidar loop back, clear buffer");  // 打印错误信息
        lidar_buffer.clear();  // 清空激光雷达缓冲区
    }
    last_timestamp_lidar = msg->header.stamp.toSec();  // 更新上次激光雷达时间戳
    
    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);  // 打印IMU和激光雷达未同步的信息
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;  // 设置时间差标志
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;  // 计算激光雷达与IMU的时间差
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);  // 打印时间差
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());  // 创建新的点云指针
    p_pre->process(msg, ptr);  // 处理点云消息
    lidar_buffer.push_back(ptr);  // 将处理后的点云添加到缓冲区
    time_buffer.push_back(last_timestamp_lidar);  // 将时间戳添加到时间缓冲区
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;  // 记录预处理时间
    mtx_buffer.unlock();  // 解锁互斥量
    sig_buffer.notify_all();  // 通知所有等待的线程
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;  // 增加发布计数
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;  // 打印IMU接收到的时间戳
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));  // 创建新的IMU消息指针

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)  // 如果激光雷达与IMU的时间差大于0.1且时间同步启用
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());  // 调整IMU消息的时间戳
    }

    double timestamp = msg->header.stamp.toSec();  // 获取IMU消息的时间戳

    mtx_buffer.lock();  // 锁定互斥量

    if (timestamp < last_timestamp_imu)  // 如果IMU时间戳小于上次IMU时间戳
    {
        ROS_WARN("imu loop back, clear buffer");  // 打印警告信息
        imu_buffer.clear();  // 清空IMU缓冲区
    }

    last_timestamp_imu = timestamp;  // 更新上次IMU时间戳

    imu_buffer.push_back(msg);  // 将IMU消息添加到缓冲区
    mtx_buffer.unlock();  // 解锁互斥量
    sig_buffer.notify_all();  // 通知所有等待的线程
}

bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {  // 如果激光雷达或IMU缓冲区为空
        return false;  // 返回false
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)  // 如果激光雷达数据未推送
    {
        meas.lidar = lidar_buffer.front();  // 获取激光雷达缓冲区的第一个元素
        if(meas.lidar->points.size() <= 1)  // 如果激光雷达点数小于等于1
        {
            lidar_buffer.pop_front();  // 弹出激光雷达缓冲区的第一个元素
            return false;  // 返回false
        }
        meas.lidar_beg_time = time_buffer.front();  // 获取时间缓冲区的第一个元素
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);  // 计算激光雷达结束时间
        lidar_pushed = true;  // 设置激光雷达数据已推送
    }

    if (last_timestamp_imu < lidar_end_time)  // 如果上次IMU时间戳小于激光雷达结束时间
    {
        return false;  // 返回false
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();  // 获取IMU缓冲区的第一个元素的时间戳
    meas.imu.clear();  // 清空测量组的IMU数据
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))  // 当IMU缓冲区不为空且IMU时间小于激光雷达结束时间
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();  // 获取IMU缓冲区的第一个元素的时间戳
        if(imu_time > lidar_end_time) break;  // 如果IMU时间大于激光雷达结束时间，跳出循环
        meas.imu.push_back(imu_buffer.front());  // 将IMU缓冲区的第一个元素添加到测量组的IMU数据
        imu_buffer.pop_front();  // 弹出IMU缓冲区的第一个元素
    }

    lidar_buffer.pop_front();  // 弹出激光雷达缓冲区的第一个元素
    time_buffer.pop_front();  // 弹出时间缓冲区的第一个元素
    lidar_pushed = false;  // 设置激光雷达数据未推送
    return true;  // 返回true
}

int process_increments = 0;  // 处理增量计数
void map_incremental()
{
    PointVector PointToAdd;  // 定义一个点向量用于存储需要添加的点
    PointVector PointNoNeedDownsample;  // 定义一个点向量用于存储不需要降采样的点
    PointToAdd.reserve(feats_down_size);  // 预留空间
    PointNoNeedDownsample.reserve(feats_down_size);  // 预留空间
    for (int i = 0; i < feats_down_size; i++)  // 遍历降采样后的特征点
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));  // 将体坐标系下的点转换到世界坐标系
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)  // 如果最近点不为空且EKF已初始化
        {
            const PointVector &points_near = Nearest_Points[i];  // 获取最近点
            bool need_add = true;  // 标记是否需要添加
            BoxPointType Box_of_Point;  // 定义一个立方体类型的变量
            PointType downsample_result, mid_point;  // 定义点类型变量
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;  // 计算中点x坐标
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;  // 计算中点y坐标
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;  // 计算中点z坐标
            float dist  = calc_dist(feats_down_world->points[i],mid_point);  // 计算距离
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);  // 如果不需要降采样，添加到不需要降采样的点向量
                continue;  // 跳过当前循环
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)  // 遍历匹配点
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;  // 如果最近点数量小于匹配点数量，跳出循环
                if (calc_dist(points_near[readd_i], mid_point) < dist)  // 如果匹配点距离小于当前距离
                {
                    need_add = false;  // 不需要添加
                    break;  // 跳出循环
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);  // 如果需要添加，添加到需要添加的点向量
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);  // 如果最近点为空或EKF未初始化，直接添加到需要添加的点向量
        }
    }

    double st_time = omp_get_wtime();  // 记录开始时间
    add_point_size = ikdtree.Add_Points(PointToAdd, true);  // 将需要添加的点添加到KD树
    ikdtree.Add_Points(PointNoNeedDownsample, false);  // 将不需要降采样的点添加到KD树
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();  // 计算添加的点数量
    kdtree_incremental_time = omp_get_wtime() - st_time;  // 计算增量时间
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));  // 定义一个点云指针用于存储待发布的点云
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());  // 定义一个点云指针用于存储待保存的点云

void publish_frame_world(const ros::Publisher &pubLaserCloudFull) //------------------------------------------------------//
{
    if(scan_pub_en)  // 如果扫描发布启用
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);  // 根据密集发布标志选择点云
        int size = laserCloudFullRes->points.size();  // 获取点云大小
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI());  // 创建新的点云指针用于显示

        // 设置显示的高度范围（可以通过launch文件配置这些参数）
        double min_display_height = -0.5;  // 最小显示高度
        double max_display_height = 1.0;   // 最大显示高度

        // 只显示指定高度范围内的点
        for (int i = 0; i < size; i++)  // 遍历点云
        {
            PointType point_world;  // 临时点
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], &point_world);  // 将体坐标系下的点转换到世界坐标系
            
            // 只添加在指定高度范围内的点
            if (point_world.z >= min_display_height && point_world.z <= max_display_height)
            {
                laserCloudWorld->points.push_back(point_world);
            }
        }

        // 设置点云大小
        laserCloudWorld->width = laserCloudWorld->points.size();
        laserCloudWorld->height = 1;

        // 发布过滤后的点云用于显示
        sensor_msgs::PointCloud2 laserCloudmsg;  // 定义点云消息
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);  // 将点云转换为ROS消息
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳
        laserCloudmsg.header.frame_id = "camera_init";  // 设置消息帧ID
        pubLaserCloudFull.publish(laserCloudmsg);  // 发布点云消息
        publish_count -= PUBFRAME_PERIOD;  // 减少发布计数
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcd_save_en)  // 如果PCD保存启用
    {
        int size = feats_undistort->points.size();  // 获取去畸变点云大小
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));  // 创建新的点云指针

        // 保存所有高度的点云
        for (int i = 0; i < size; i++)  // 遍历点云
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);  // 将体坐标系下的点转换到世界坐标系
        }
        *pcl_wait_save += *laserCloudWorld;  // 将完整点云添加到待保存的点云
    }
}


void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();  // 获取去畸变点云的大小
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));  // 创建新的点云指针用于存储IMU坐标系下的点云

    for (int i = 0; i < size; i++)  // 遍历去畸变点云
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);  // 将体坐标系下的点转换到IMU坐标系
    }

    sensor_msgs::PointCloud2 laserCloudmsg;  // 定义点云消息
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);  // 将点云转换为ROS消息
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳
    laserCloudmsg.header.frame_id = "body";  // 设置消息帧ID为"body"
    pubLaserCloudFull_body.publish(laserCloudmsg);  // 发布点云消息
    publish_count -= PUBFRAME_PERIOD;  // 减少发布计数
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));  // 创建新的点云指针用于存储世界坐标系下的点云
    for (int i = 0; i < effct_feat_num; i++)  // 遍历有效特征点
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);  // 将体坐标系下的点转换到世界坐标系
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;  // 定义点云消息
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);  // 将点云转换为ROS消息
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳
    laserCloudFullRes3.header.frame_id = "camera_init";  // 设置消息帧ID为"camera_init"
    pubLaserCloudEffect.publish(laserCloudFullRes3);  // 发布点云消息
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;  // 定义点云消息
    pcl::toROSMsg(*featsFromMap, laserCloudMap);  // 将地图特征点云转换为ROS消息
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳
    laserCloudMap.header.frame_id = "camera_init";  // 设置消息帧ID为"camera_init"
    pubLaserCloudMap.publish(laserCloudMap);  // 发布点云消息
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);  // 设置位姿的x坐标
    out.pose.position.y = state_point.pos(1);  // 设置位姿的y坐标
    out.pose.position.z = state_point.pos(2);  // 设置位姿的z坐标
    out.pose.orientation.x = geoQuat.x;  // 设置位姿的四元数x分量
    out.pose.orientation.y = geoQuat.y;  // 设置位姿的四元数y分量
    out.pose.orientation.z = geoQuat.z;  // 设置位姿的四元数z分量
    out.pose.orientation.w = geoQuat.w;  // 设置位姿的四元数w分量
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";  // 设置里程计消息的帧ID为"camera_init"
    odomAftMapped.child_frame_id = "body";  // 设置里程计消息的子帧ID为"body"
    // odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳为激光雷达结束时间
    odomAftMapped.header.stamp = ros::Time::now();  // 设置消息时间戳为当前时间
    set_posestamp(odomAftMapped.pose);  // 设置里程计的位姿
    pubOdomAftMapped.publish(odomAftMapped);  // 发布里程计消息
    auto P = kf.get_P();  // 获取卡尔曼滤波器的协方差矩阵
    for (int i = 0; i < 6; i ++)  // 遍历协方差矩阵的前6行
    {
        int k = i < 3 ? i + 3 : i - 3;  // 计算索引k
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);  // 设置协方差矩阵的值
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);  // 设置协方差矩阵的值
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);  // 设置协方差矩阵的值
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);  // 设置协方差矩阵的值
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);  // 设置协方差矩阵的值
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);  // 设置协方差矩阵的值
    }

    static tf::TransformBroadcaster br;  // 定义静态变换广播器
    tf::Transform transform;  // 定义变换
    tf::Quaternion q;  // 定义四元数
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));  // 设置变换的原点
    q.setW(odomAftMapped.pose.pose.orientation.w);  // 设置四元数的w分量
    q.setX(odomAftMapped.pose.pose.orientation.x);  // 设置四元数的x分量
    q.setY(odomAftMapped.pose.pose.orientation.y);  // 设置四元数的y分量
    q.setZ(odomAftMapped.pose.pose.orientation.z);  // 设置四元数的z分量
    transform.setRotation(q);  // 设置变换的旋转
    // TODO 这里使用当前时间发布tf 否则当livox时间不正确时无法正常tf
    // br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_init", "body"));  // 发布变换
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);  // 设置体姿态消息的位姿
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);  // 设置消息时间戳
    msg_body_pose.header.frame_id = "camera_init";  // 设置消息帧ID为"camera_init"

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;  // 定义静态计数器
    jjj++;  // 增加计数器
    if (jjj % 10 == 0)  // 每10次执行一次
    {
        path.poses.push_back(msg_body_pose);  // 将体姿态消息添加到路径中
        pubPath.publish(path);  // 发布路径消息
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();  // 记录匹配开始时间
    laserCloudOri->clear();  // 清空原始激光点云
    corr_normvect->clear();  // 清空校正后的法向量
    total_residual = 0.0;  // 初始化总残差

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);  // 设置OpenMP线程数
        #pragma omp parallel for  // 并行for循环
    #endif
    for (int i = 0; i < feats_down_size; i++)  // 遍历降采样后的特征点
    {
        PointType &point_body  = feats_down_body->points[i];  // 获取体坐标系下的点
        PointType &point_world = feats_down_world->points[i];  // 获取世界坐标系下的点

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);  // 将体坐标系下的点转换为V3D类型
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);  // 计算全局坐标
        point_world.x = p_global(0);  // 设置世界坐标系下的x坐标
        point_world.y = p_global(1);  // 设置世界坐标系下的y坐标
        point_world.z = p_global(2);  // 设置世界坐标系下的z坐标
        point_world.intensity = point_body.intensity;  // 保留输入点的强度

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);  // 定义一个向量用于存储最近点的平方距离

        auto &points_near = Nearest_Points[i];  // 获取最近点

        if (ekfom_data.converge)  // 如果EKF已收敛
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);  // 在地图中查找最近的表面
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;  // 判断是否选择该表面
        }

        if (!point_selected_surf[i]) continue;  // 如果未选择该表面，跳过当前循环

        VF(4) pabcd;  // 定义一个向量用于存储平面参数
        point_selected_surf[i] = false;  // 初始化选择标志
        if (esti_plane(pabcd, points_near, 0.1f))  // 估计平面
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);  // 计算点到平面的距离
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());  // 计算权重

            if (s > 0.9)  // 如果权重大于0.9
            {
                point_selected_surf[i] = true;  // 选择该表面
                normvec->points[i].x = pabcd(0);  // 设置法向量的x分量
                normvec->points[i].y = pabcd(1);  // 设置法向量的y分量
                normvec->points[i].z = pabcd(2);  // 设置法向量的z分量
                normvec->points[i].intensity = pd2;  // 设置法向量的强度
                res_last[i] = abs(pd2);  // 记录残差
            }
        }
    }
    
    effct_feat_num = 0;  // 初始化有效特征点数量

    for (int i = 0; i < feats_down_size; i++)  // 遍历降采样后的特征点
    {
        if (point_selected_surf[i])  // 如果选择了该表面
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];  // 将体坐标系下的点添加到原始激光点云
            corr_normvect->points[effct_feat_num] = normvec->points[i];  // 将法向量添加到校正后的法向量
            total_residual += res_last[i];  // 累加残差
            effct_feat_num ++;  // 增加有效特征点数量
        }
    }

    res_mean_last = total_residual / effct_feat_num;  // 计算平均残差
    match_time  += omp_get_wtime() - match_start;  // 计算匹配时间
    double solve_start_  = omp_get_wtime();  // 记录求解开始时间
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12);  // 初始化雅可比矩阵
    ekfom_data.h.resize(effct_feat_num);  // 调整测量向量的大小

    for (int i = 0; i < effct_feat_num; i++)  // 遍历有效特征点
    {
        const PointType &laser_p  = laserCloudOri->points[i];  // 获取原始激光点
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);  // 将点转换为V3D类型
        M3D point_be_crossmat;  // 定义交叉矩阵
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);  // 计算交叉矩阵
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;  // 计算点在IMU坐标系下的坐标
        M3D point_crossmat;  // 定义交叉矩阵
        point_crossmat<<SKEW_SYM_MATRX(point_this);  // 计算交叉矩阵

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];  // 获取法向量
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);  // 将法向量转换为V3D类型

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);  // 计算C向量
        V3D A(point_crossmat * C);  // 计算A向量
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);  // 计算B向量
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);  // 设置雅可比矩阵的值

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;  // 设置测量向量的值
    }
    solve_time += omp_get_wtime() - solve_start_;  // 计算求解时间
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");  // 初始化ROS节点，节点名为"laserMapping"
    ros::NodeHandle nh;  // 创建节点句柄

    nh.param<bool>("publish/scan_publish_en",scan_pub_en,1);  // 从参数服务器获取scan_publish_en参数
    nh.param<bool>("publish/dense_publish_en",dense_pub_en,1);  // 从参数服务器获取dense_publish_en参数
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en,1);  // 从参数服务器获取scan_bodyframe_pub_en参数
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);  // 从参数服务器获取max_iteration参数
    nh.param<string>("map_file_path",map_file_path,"");  // 从参数服务器获取map_file_path参数
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");  // 从参数服务器获取lid_topic参数
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");  // 从参数服务器获取imu_topic参数
    nh.param<bool>("common/time_sync_en", time_sync_en, false);  // 从参数服务器获取time_sync_en参数
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);  // 从参数服务器获取filter_size_corner参数
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);  // 从参数服务器获取filter_size_surf参数
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);  // 从参数服务器获取filter_size_map参数
    nh.param<double>("cube_side_length",cube_len,200);  // 从参数服务器获取cube_side_length参数
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);  // 从参数服务器获取det_range参数
    nh.param<double>("mapping/fov_degree",fov_deg,180);  // 从参数服务器获取fov_degree参数
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);  // 从参数服务器获取gyr_cov参数
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);  // 从参数服务器获取acc_cov参数
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);  // 从参数服务器获取b_gyr_cov参数
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);  // 从参数服务器获取b_acc_cov参数
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);  // 从参数服务器获取blind参数
    nh.param<double>("preprocess/max_range",p_pre->max_scan_range, 100.f);  // 从参数服务器获取max_range参数
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);  // 从参数服务器获取lidar_type参数
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);  // 从参数服务器获取scan_line参数
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);  // 从参数服务器获取point_filter_num参数
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, 0);  // 从参数服务器获取feature_extract_enable参数
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);  // 从参数服务器获取runtime_pos_log_enable参数
    nh.param<bool>("pcd_save_enable", pcd_save_en, 0);  // 从参数服务器获取pcd_save_enable参数
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());  // 从参数服务器获取extrinsic_T参数
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());  // 从参数服务器获取extrinsic_R参数
    nh.param<double>("min_display_height", min_display_height, -0.5);  // 从参数服务器读取最小显示高度----------------------------------------------
    nh.param<double>("max_display_height", max_display_height, 1.0);   // 从参数服务器读取最大显示高度----------------------------------------------
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;  // 打印雷达类型
    
    path.header.stamp    = ros::Time::now();  // 设置路径消息的时间戳
    path.header.frame_id ="camera_init";  // 设置路径消息的帧ID

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;  // 定义有效特征点数量和帧数
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;  // 定义时间变量
    bool flg_EKF_converged, EKF_stop_flg = 0;  // 定义EKF收敛标志和停止标志
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);  // 计算视场角
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);  // 计算半视场角的余弦值

    _featsArray.reset(new PointCloudXYZI());  // 重置特征数组

    memset(point_selected_surf, true, sizeof(point_selected_surf));  // 初始化选中的表面点
    memset(res_last, -1000.0f, sizeof(res_last));  // 初始化上一次的残差
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);  // 设置表面降采样滤波器的叶子大小
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);  // 设置地图降采样滤波器的叶子大小
    memset(point_selected_surf, true, sizeof(point_selected_surf));  // 初始化选中的表面点
    memset(res_last, -1000.0f, sizeof(res_last));  // 初始化上一次的残差

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);  // 设置激光雷达相对于IMU的平移
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);  // 设置激光雷达相对于IMU的旋转
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);  // 设置IMU的外参
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));  // 设置陀螺仪的协方差
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));  // 设置加速度计的协方差
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));  // 设置陀螺仪偏置的协方差
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));  // 设置加速度计偏置的协方差

    double epsi[23] = {0.001};  // 定义一个误差数组
    fill(epsi, epsi+23, 0.001);  // 填充误差数组
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);  // 初始化扩展卡尔曼滤波器

    /*** debug record ***/
    FILE *fp;  // 定义文件指针
    string pos_log_dir = root_dir + "/Log/pos_log.txt";  // 定义位置日志目录
    fp = fopen(pos_log_dir.c_str(),"w");  // 打开位置日志文件

    ofstream fout_pre, fout_out, fout_dbg;  // 定义输出文件流
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);  // 打开预处理矩阵文件
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);  // 打开输出矩阵文件
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);  // 打开调试文件
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;  // 如果文件打开成功，打印信息
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;  // 如果文件不存在，打印信息

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);  // 根据雷达类型订阅点云话题
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);  // 订阅IMU话题
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);  // 广播完整点云
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);  // 广播体坐标系下的完整点云
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);  // 广播有效点云
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);  // 广播激光地图
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/Odometry", 100000);  // 广播里程计
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 100000);  // 广播路径
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);  // 注册信号处理函数
    ros::Rate rate(5000);  // 设置循环频率
    bool status = ros::ok();  // 检查ROS状态
    while (status)  // 循环执行
    {
        if (flg_exit) break;  // 如果退出标志为真，跳出循环
        ros::spinOnce();  // 处理一次回调
        if(sync_packages(Measures))  // 如果同步成功
        {
            if (flg_reset)  // 如果重置标志为真
            {
                ROS_WARN("reset when rosbag play back");  // 打印警告信息
                p_imu->Reset();  // 重置IMU
                flg_reset = false;  // 重置标志置为假
                Measures.imu.clear();  // 清空IMU数据
                continue;  // 继续下一次循环
            }

            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;  // 定义时间变量

            match_time = 0;  // 初始化匹配时间
            kdtree_search_time = 0.0;  // 初始化KD树搜索时间
            solve_time = 0;  // 初始化求解时间
            solve_const_H_time = 0;  // 初始化常数H求解时间
            svd_time   = 0;  // 初始化SVD时间
            t0 = omp_get_wtime();  // 记录开始时间

            p_imu->Process(Measures, kf, feats_undistort);  // 处理IMU数据
            state_point = kf.get_x();  // 获取状态点
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;  // 计算激光雷达位置

            if (feats_undistort->empty() || (feats_undistort == NULL))  // 如果去畸变特征为空
            {
                first_lidar_time = Measures.lidar_beg_time;  // 设置第一次激光雷达时间
                p_imu->first_lidar_time = first_lidar_time;  // 设置IMU的第一次激光雷达时间
                // cout<<"FAST-LIO not ready"<<endl;  // 打印信息
                continue;  // 继续下一次循环
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;  // 判断EKF是否初始化
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();  // 分割激光雷达视场内的地图

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);  // 设置输入点云
            downSizeFilterSurf.filter(*feats_down_body);  // 过滤点云
            t1 = omp_get_wtime();  // 记录时间
            feats_down_size = feats_down_body->points.size();  // 获取降采样后的特征点数量
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)  // 如果KD树根节点为空
            {
                if(feats_down_size > 5)  // 如果特征点数量大于5
                {
                    ikdtree.set_downsample_param(filter_size_map_min);  // 设置降采样参数
                    feats_down_world->resize(feats_down_size);  // 调整世界坐标系下的特征点大小
                    for(int i = 0; i < feats_down_size; i++)  // 遍历特征点
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));  // 将体坐标系下的点转换到世界坐标系
                    }
                    ikdtree.Build(feats_down_world->points);  // 构建KD树
                }
                continue;  // 继续下一次循环
            }
            int featsFromMapNum = ikdtree.validnum();  // 获取地图中的特征点数量
            kdtree_size_st = ikdtree.size();  // 获取KD树大小
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;  // 打印信息

            /*** ICP and iterated Kalman filter update ***/
            normvec->resize(feats_down_size);  // 调整法向量大小
            feats_down_world->resize(feats_down_size);  // 调整世界坐标系下的特征点大小

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);  // 计算外部欧拉角
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;  // 记录预处理信息

            if(0) // 如果需要查看地图点，将其改为"if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);  // 交换KD树的存储
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);  // 展平KD树
                featsFromMap->clear();  // 清空从地图中提取的特征
                featsFromMap->points = ikdtree.PCL_Storage;  // 将KD树存储的点赋值给地图特征
            }

            pointSearchInd_surf.resize(feats_down_size);  // 调整表面点搜索索引的大小
            Nearest_Points.resize(feats_down_size);  // 调整最近点的大小
            int  rematch_num = 0;  // 初始化重匹配次数
            bool nearest_search_en = true;  // 启用最近点搜索

            t2 = omp_get_wtime();  // 记录时间
            
            /*** 迭代状态估计 ***/
            double t_update_start = omp_get_wtime();  // 记录更新开始时间
            double solve_H_time = 0;  // 初始化求解H矩阵的时间
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);  // 更新扩展卡尔曼滤波器
            state_point = kf.get_x();  // 获取状态点
            euler_cur = SO3ToEuler(state_point.rot);  // 计算当前欧拉角
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;  // 计算激光雷达位置
            geoQuat.x = state_point.rot.coeffs()[0];  // 设置四元数x分量
            geoQuat.y = state_point.rot.coeffs()[1];  // 设置四元数y分量
            geoQuat.z = state_point.rot.coeffs()[2];  // 设置四元数z分量
            geoQuat.w = state_point.rot.coeffs()[3];  // 设置四元数w分量

            double t_update_end = omp_get_wtime();  // 记录更新结束时间

            /******* 发布里程计 *******/
            publish_odometry(pubOdomAftMapped);  // 发布里程计消息

            /*** 将特征点添加到地图KD树 ***/
            t3 = omp_get_wtime();  // 记录时间
            map_incremental();  // 增量更新地图
            t5 = omp_get_wtime();  // 记录时间
            
            /******* 发布点云 *******/
            publish_path(pubPath);  // 发布路径消息
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);  // 发布世界坐标系下的点云
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);  // 发布体坐标系下的点云
            // publish_effect_world(pubLaserCloudEffect);  // 发布有效点云
            // publish_map(pubLaserCloudMap);  // 发布地图

            /*** 调试变量 ***/
            if (runtime_pos_log)  // 如果启用运行时位置日志
            {
                frame_num ++;  // 增加帧数
                kdtree_size_end = ikdtree.size();  // 获取KD树大小
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;  // 计算平均消耗时间
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;  // 计算平均ICP时间
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;  // 计算平均匹配时间
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;  // 计算平均增量时间
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;  // 计算平均求解时间
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;  // 计算平均构造H矩阵时间
                T1[time_log_counter] = Measures.lidar_beg_time;  // 记录时间戳
                s_plot[time_log_counter] = t5 - t0;  // 记录总时间
                s_plot2[time_log_counter] = feats_undistort->points.size();  // 记录去畸变点云大小
                s_plot3[time_log_counter] = kdtree_incremental_time;  // 记录增量时间
                s_plot4[time_log_counter] = kdtree_search_time;  // 记录搜索时间
                s_plot5[time_log_counter] = kdtree_delete_counter;  // 记录删除计数
                s_plot6[time_log_counter] = kdtree_delete_time;  // 记录删除时间
                s_plot7[time_log_counter] = kdtree_size_st;  // 记录KD树初始大小
                s_plot8[time_log_counter] = kdtree_size_end;  // 记录KD树结束大小
                s_plot9[time_log_counter] = aver_time_consu;  // 记录平均消耗时间
                s_plot10[time_log_counter] = add_point_size;  // 记录添加点大小
                time_log_counter ++;  // 增加时间日志计数
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);  // 打印调试信息
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);  // 计算外部欧拉角
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;  // 记录输出信息
                dump_lio_state_to_log(fp);  // 记录LIO状态到日志
            }
        }

        status = ros::ok();  // 检查ROS状态
        rate.sleep();  // 休眠以保持循环频率
    }

    /**************** 保存地图 ****************/
    /* 1. 确保有足够的内存
    /* 2. PCD保存会极大影响实时性能 **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)  // 如果待保存点云大小大于0且启用PCD保存
    {
        string file_name = string("scans.pcd");  // 定义文件名
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);  // 定义文件路径
        pcl::PCDWriter pcd_writer;  // 定义PCD写入器
        cout << "current scan saved to /PCD/" << file_name<<endl;  // 打印保存信息
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);  // 写入PCD文件
    }

    fout_out.close();  // 关闭输出文件
    fout_pre.close();  // 关闭预处理文件

    if (runtime_pos_log)  // 如果启用运行时位置日志
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;  // 定义向量用于存储日志数据    
        FILE *fp2;  // 定义文件指针
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";  // 定义日志目录
        fp2 = fopen(log_dir.c_str(),"w");  // 打开日志文件
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");  // 写入日志头
        for (int i = 0;i<time_log_counter; i++){  // 遍历时间日志计数
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);  // 写入日志数据
            t.push_back(T1[i]);  // 添加时间戳
            s_vec.push_back(s_plot9[i]);  // 添加平均消耗时间
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);  // 添加增量时间和删除时间
            s_vec3.push_back(s_plot4[i]);  // 添加搜索时间
            s_vec5.push_back(s_plot[i]);  // 添加总时间
        }
        fclose(fp2);  // 关闭日志文件
    }

    return 0;  // 返回0，表示程序正常结束
}

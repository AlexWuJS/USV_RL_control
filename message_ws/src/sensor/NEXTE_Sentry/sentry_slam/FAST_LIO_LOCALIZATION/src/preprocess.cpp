#include "preprocess.h"  // 包含预处理头文件

#define RETURN0     0x00  // 定义返回值0
#define RETURN0AND1 0x10  // 定义返回值16

Preprocess::Preprocess()
  :feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1)  // 初始化成员变量
{
  inf_bound = 10;  // 设置信息边界
  N_SCANS   = 6;  // 设置扫描线数量
  group_size = 8;  // 设置组大小
  disA = 0.01;  // 设置距离A
  disA = 0.1; // B?  // 设置距离A（可能是B）
  p2l_ratio = 225;  // 设置点到线的比例
  limit_maxmid =6.25;  // 设置最大中间限制
  limit_midmin =6.25;  // 设置中间最小限制
  limit_maxmin = 3.24;  // 设置最大最小限制
  jump_up_limit = 170.0;  // 设置跳跃上限
  jump_down_limit = 8.0;  // 设置跳跃下限
  cos160 = 160.0;  // 设置160度的余弦值
  edgea = 2;  // 设置边缘a
  edgeb = 0.1;  // 设置边缘b
  smallp_intersect = 172.5;  // 设置小p交点
  smallp_ratio = 1.2;  // 设置小p比例
  given_offset_time = false;  // 设置给定偏移时间标志

  jump_up_limit = cos(jump_up_limit/180*M_PI);  // 计算跳跃上限的余弦值
  jump_down_limit = cos(jump_down_limit/180*M_PI);  // 计算跳跃下限的余弦值
  cos160 = cos(cos160/180*M_PI);  // 计算160度的余弦值
  smallp_intersect = cos(smallp_intersect/180*M_PI);  // 计算小p交点的余弦值
}

Preprocess::~Preprocess() {}  // 析构函数

void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num)
{
  feature_enabled = feat_en;  // 设置特征启用标志
  lidar_type = lid_type;  // 设置雷达类型
  blind = bld;  // 设置盲区
  point_filter_num = pfilt_num;  // 设置点过滤数量
}

void Preprocess::process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{  
  avia_handler(msg);  // 调用avia处理函数
  *pcl_out = pl_surf;  // 将处理后的点云赋值给输出点云
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out)
{
  switch (lidar_type)  // 根据雷达类型选择处理函数
  {
  case OUST64:
    oust64_handler(msg);  // 调用OUST64处理函数
    break;

  case VELO16:
    velodyne_handler(msg);  // 调用Velodyne处理函数
    break;
  
  default:
    printf("Error LiDAR Type");  // 打印错误信息
    break;
  }
  *pcl_out = pl_surf;  // 将处理后的点云赋值给输出点云
}

void Preprocess::avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg)
{
  pl_surf.clear();  // 清空表面点云
  pl_corn.clear();  // 清空角点云
  pl_full.clear();  // 清空完整点云
  double t1 = omp_get_wtime();  // 记录时间
  int plsize = msg->point_num;  // 获取点云大小
  // cout<<"plsie: "<<plsize<<endl;

  pl_corn.reserve(plsize);  // 预留角点云空间
  pl_surf.reserve(plsize);  // 预留表面点云空间
  pl_full.resize(plsize);  // 调整完整点云大小

  for(int i=0; i<N_SCANS; i++)  // 遍历扫描线
  {
    pl_buff[i].clear();  // 清空缓冲区
    pl_buff[i].reserve(plsize);  // 预留缓冲区空间
  }
  uint valid_num = 0;  // 初始化有效点数量
  
  if (feature_enabled)  // 如果启用特征提取
  {
    for(uint i=1; i<plsize; i++)  // 遍历点云
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))  // 检查点的线号和标签
      {
        pl_full[i].x = msg->points[i].x;  // 设置点的x坐标
        pl_full[i].y = msg->points[i].y;  // 设置点的y坐标
        pl_full[i].z = msg->points[i].z;  // 设置点的z坐标
        pl_full[i].intensity = msg->points[i].reflectivity;  // 设置点的反射强度
        pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 使用曲率作为每个激光点的时间

        bool is_new = false;  // 初始化新点标志
        if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
            || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
            || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7))  // 检查点是否为新点
        {
          pl_buff[msg->points[i].line].push_back(pl_full[i]);  // 将点添加到缓冲区
        }
      }
    }
    static int count = 0;  // 静态计数器
    static double time = 0.0;  // 静态时间变量
    count ++;  // 增加计数器
    double t0 = omp_get_wtime();  // 记录时间
    for(int j=0; j<N_SCANS; j++)  // 遍历扫描线
    {
      if(pl_buff[j].size() <= 5) continue;  // 如果缓冲区大小小于等于5，跳过
      pcl::PointCloud<PointType> &pl = pl_buff[j];  // 获取缓冲区点云
      plsize = pl.size();  // 获取点云大小
      vector<orgtype> &types = typess[j];  // 获取类型向量
      types.clear();  // 清空类型向量
      types.resize(plsize);  // 调整类型向量大小
      plsize--;  // 减少点云大小
      for(uint i=0; i<plsize; i++)  // 遍历点云
      {
        types[i].range = pl[i].x * pl[i].x + pl[i].y * pl[i].y;  // 计算点的距离
        vx = pl[i].x - pl[i + 1].x;  // 计算x方向的差值
        vy = pl[i].y - pl[i + 1].y;  // 计算y方向的差值
        vz = pl[i].z - pl[i + 1].z;  // 计算z方向的差值
        types[i].dista = vx * vx + vy * vy + vz * vz;  // 计算距离平方
      }
      types[plsize].range = pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y;  // 计算最后一个点的距离
      give_feature(pl, types);  // 提取特征
      // pl_surf += pl;
    }
    time += omp_get_wtime() - t0;  // 计算特征提取时间
    printf("Feature extraction time: %lf \n", time / count);  // 打印特征提取时间
  }
  else  // 如果未启用特征提取
  {
    for(uint i=1; i<plsize; i++)  // 遍历点云
    {
      if((msg->points[i].line < N_SCANS) && ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00))  // 检查点的线号和标签
      {
        valid_num ++;  // 增加有效点数量
        if (valid_num % point_filter_num == 0)  // 如果有效点数量满足过滤条件
        {
          pl_full[i].x = msg->points[i].x;  // 设置点的x坐标
          pl_full[i].y = msg->points[i].y;  // 设置点的y坐标
          pl_full[i].z = msg->points[i].z;  // 设置点的z坐标
          pl_full[i].intensity = msg->points[i].reflectivity;  // 设置点的反射强度
          pl_full[i].curvature = msg->points[i].offset_time / float(1000000); // 使用曲率作为每个激光点的时间

          if((abs(pl_full[i].x - pl_full[i-1].x) > 1e-7) 
              || (abs(pl_full[i].y - pl_full[i-1].y) > 1e-7)
              || (abs(pl_full[i].z - pl_full[i-1].z) > 1e-7)
              && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y > blind))  // 检查点是否为新点且不在盲区内
          {
            if(pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y < max_scan_range * max_scan_range){  // 检查点是否在最大扫描范围内
                pl_surf.push_back(pl_full[i]);  // 将点添加到表面点云
            }
          }
        }
      }
    }
  }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pl_surf.clear();  // 清空表面点云
  pl_corn.clear();  // 清空角点云
  pl_full.clear();  // 清空完整点云
  pcl::PointCloud<ouster_ros::Point> pl_orig;  // 定义原始点云
  pcl::fromROSMsg(*msg, pl_orig);  // 从ROS消息转换为点云
  int plsize = pl_orig.size();  // 获取点云大小
  pl_corn.reserve(plsize);  // 预留角点云空间
  pl_surf.reserve(plsize);  // 预留表面点云空间
  if (feature_enabled)  // 如果启用特征提取
  {
    for (int i = 0; i < N_SCANS; i++)  // 遍历扫描线
    {
      pl_buff[i].clear();  // 清空缓冲区
      pl_buff[i].reserve(plsize);  // 预留缓冲区空间
    }

    for (uint i = 0; i < plsize; i++)  // 遍历点云
    {
      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;  // 计算点的距离
      if (range < blind) continue;  // 如果点在盲区内，跳过
      Eigen::Vector3d pt_vec;  // 定义三维向量
      PointType added_pt;  // 定义点类型
      added_pt.x = pl_orig.points[i].x;  // 设置点的x坐标
      added_pt.y = pl_orig.points[i].y;  // 设置点的y坐标
      added_pt.z = pl_orig.points[i].z;  // 设置点的z坐标
      added_pt.intensity = pl_orig.points[i].intensity;  // 设置点的反射强度
      added_pt.normal_x = 0;  // 设置点的法向量x分量
      added_pt.normal_y = 0;  // 设置点的法向量y分量
      added_pt.normal_z = 0;  // 设置点的法向量z分量
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;  // 计算偏航角
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;  // 调整偏航角
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;  // 调整偏航角

      added_pt.curvature = pl_orig.points[i].t / 1e6;  // 设置点的曲率

      if(pl_orig.points[i].ring < N_SCANS)  // 如果点的环号小于扫描线数量
      {
        pl_buff[pl_orig.points[i].ring].push_back(added_pt);  // 将点添加到缓冲区
      }
    }

    for (int j = 0; j < N_SCANS; j++)  // 遍历扫描线
    {
      PointCloudXYZI &pl = pl_buff[j];  // 获取缓冲区点云
      int linesize = pl.size();  // 获取点云大小
      vector<orgtype> &types = typess[j];  // 获取类型向量
      types.clear();  // 清空类型向量
      types.resize(linesize);  // 调整类型向量大小
      linesize--;  // 减少点云大小
      for (uint i = 0; i < linesize; i++)  // 遍历点云
      {
        types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 计算点的距离
        vx = pl[i].x - pl[i + 1].x;  // 计算x方向的差值
        vy = pl[i].y - pl[i + 1].y;  // 计算y方向的差值
        vz = pl[i].z - pl[i + 1].z;  // 计算z方向的差值
        types[i].dista = vx * vx + vy * vy + vz * vz;  // 计算距离平方
      }
      types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);  // 计算最后一个点的距离
      give_feature(pl, types);  // 提取特征
    }
  }
  else  // 如果未启用特征提取
  {
    double time_stamp = msg->header.stamp.toSec();  // 获取时间戳
    // cout << "===================================" << endl;
    // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
    for (int i = 0; i < pl_orig.points.size(); i++)  // 遍历点云
    {
      if (i % point_filter_num != 0) continue;  // 如果点不满足过滤条件，跳过

      double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y + pl_orig.points[i].z * pl_orig.points[i].z;  // 计算点的距离
      
      if (range < blind) continue;  // 如果点在盲区内，跳过
      
      Eigen::Vector3d pt_vec;  // 定义三维向量
      PointType added_pt;  // 定义点类型
      added_pt.x = pl_orig.points[i].x;  // 设置点的x坐标
      added_pt.y = pl_orig.points[i].y;  // 设置点的y坐标
      added_pt.z = pl_orig.points[i].z;  // 设置点的z坐标
      added_pt.intensity = pl_orig.points[i].intensity;  // 设置点的反射强度
      added_pt.normal_x = 0;  // 设置点的法向量x分量
      added_pt.normal_y = 0;  // 设置点的法向量y分量
      added_pt.normal_z = 0;  // 设置点的法向量z分量
      double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;  // 计算偏航角
      if (yaw_angle >= 180.0)
        yaw_angle -= 360.0;  // 调整偏航角
      if (yaw_angle <= -180.0)
        yaw_angle += 360.0;  // 调整偏航角

      added_pt.curvature = pl_orig.points[i].t / 1e6;  // 设置点的曲率

      pl_surf.points.push_back(added_pt);  // 将点添加到表面点云
    }
  }
  // pub_func(pl_surf, pub_full, msg->header.stamp);
  // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    pl_surf.clear();  // 清空表面点云
    pl_corn.clear();  // 清空角点云
    pl_full.clear();  // 清空完整点云

    pcl::PointCloud<velodyne_ros::Point> pl_orig;  // 定义原始点云
    pcl::fromROSMsg(*msg, pl_orig);  // 从ROS消息转换为点云
    int plsize = pl_orig.points.size();  // 获取点云大小
    pl_surf.reserve(plsize);  // 预留表面点云空间

    /*** 这些变量仅在没有给定点时间戳时有效 ***/
    double omega_l=3.61;  // 扫描角速度
    std::vector<bool> is_first(N_SCANS,true);  // 标记每条扫描线的第一个点
    std::vector<double> yaw_fp(N_SCANS, 0.0);  // 每条扫描线第一个点的偏航角
    std::vector<float> yaw_last(N_SCANS, 0.0);  // 每条扫描线最后一个点的偏航角
    std::vector<float> time_last(N_SCANS, 0.0);  // 每条扫描线最后一个点的时间偏移
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0)  // 如果点云中最后一个点的时间大于0
    {
      given_offset_time = true;  // 设置给定偏移时间标志为真
    }
    else
    {
      given_offset_time = false;  // 设置给定偏移时间标志为假
      double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;  // 计算第一个点的偏航角
      double yaw_end  = yaw_first;  // 初始化最后一个点的偏航角
      int layer_first = pl_orig.points[0].ring;  // 获取第一个点的环号
      for (uint i = plsize - 1; i > 0; i--)  // 从最后一个点向前遍历
      {
        if (pl_orig.points[i].ring == layer_first)  // 如果点的环号与第一个点相同
        {
          yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;  // 计算最后一个点的偏航角
          break;  // 退出循环
        }
      }
    }

    if(feature_enabled)  // 如果启用特征提取
    {
      for (int i = 0; i < N_SCANS; i++)  // 遍历扫描线
      {
        pl_buff[i].clear();  // 清空缓冲区
        pl_buff[i].reserve(plsize);  // 预留缓冲区空间
      }
      
      for (int i = 0; i < plsize; i++)  // 遍历点云
      {
        PointType added_pt;  // 定义点类型
        added_pt.normal_x = 0;  // 设置点的法向量x分量
        added_pt.normal_y = 0;  // 设置点的法向量y分量
        added_pt.normal_z = 0;  // 设置点的法向量z分量
        int layer  = pl_orig.points[i].ring;  // 获取点的环号
        if (layer >= N_SCANS) continue;  // 如果环号大于等于扫描线数量，跳过
        added_pt.x = pl_orig.points[i].x;  // 设置点的x坐标
        added_pt.y = pl_orig.points[i].y;  // 设置点的y坐标
        added_pt.z = pl_orig.points[i].z;  // 设置点的z坐标
        added_pt.intensity = pl_orig.points[i].intensity;  // 设置点的反射强度
        added_pt.curvature = pl_orig.points[i].time / 1000.0; // 设置点的曲率，单位为毫秒

        if (!given_offset_time)  // 如果没有给定偏移时间
        {
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 计算偏航角
          if (is_first[layer])  // 如果是该层的第一个点
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;  // 设置该层的第一个点的偏航角
              is_first[layer]=false;  // 设置该层的第一个点标志为假
              added_pt.curvature = 0.0;  // 设置曲率为0
              yaw_last[layer]=yaw_angle;  // 设置该层的最后一个点的偏航角
              time_last[layer]=added_pt.curvature;  // 设置该层的最后一个点的时间偏移
              continue;  // 继续下一个点
          }

          if (yaw_angle <= yaw_fp[layer])  // 如果当前点的偏航角小于等于第一个点的偏航角
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;  // 计算时间偏移
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;  // 计算时间偏移
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;  // 如果时间偏移小于最后一个点的时间偏移，调整时间偏移

          yaw_last[layer] = yaw_angle;  // 更新最后一个点的偏航角
          time_last[layer]=added_pt.curvature;  // 更新最后一个点的时间偏移
        }

        pl_buff[layer].points.push_back(added_pt);  // 将点添加到缓冲区
      }

      for (int j = 0; j < N_SCANS; j++)  // 遍历扫描线
      {
        PointCloudXYZI &pl = pl_buff[j];  // 获取缓冲区点云
        int linesize = pl.size();  // 获取点云大小
        if (linesize < 2) continue;  // 如果点云大小小于2，跳过
        vector<orgtype> &types = typess[j];  // 获取类型向量
        types.clear();  // 清空类型向量
        types.resize(linesize);  // 调整类型向量大小
        linesize--;  // 减少点云大小
        for (uint i = 0; i < linesize; i++)  // 遍历点云
        {
          types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);  // 计算点的距离
          vx = pl[i].x - pl[i + 1].x;  // 计算x方向的差值
          vy = pl[i].y - pl[i + 1].y;  // 计算y方向的差值
          vz = pl[i].z - pl[i + 1].z;  // 计算z方向的差值
          types[i].dista = vx * vx + vy * vy + vz * vz;  // 计算距离平方
        }
        types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);  // 计算最后一个点的距离
        give_feature(pl, types);  // 提取特征
      }
    }
    else  // 如果未启用特征提取
    {
      for (int i = 0; i < plsize; i++)  // 遍历点云
      {
        PointType added_pt;  // 定义点类型
        // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;
        
        added_pt.normal_x = 0;  // 设置点的法向量x分量
        added_pt.normal_y = 0;  // 设置点的法向量y分量
        added_pt.normal_z = 0;  // 设置点的法向量z分量
        added_pt.x = pl_orig.points[i].x;  // 设置点的x坐标
        added_pt.y = pl_orig.points[i].y;  // 设置点的y坐标
        added_pt.z = pl_orig.points[i].z;  // 设置点的z坐标
        added_pt.intensity = pl_orig.points[i].intensity;  // 设置点的反射强度
        added_pt.curvature = pl_orig.points[i].time / 1000.0;  // 设置点的曲率

        if (!given_offset_time)  // 如果没有给定偏移时间
        {
          int layer = pl_orig.points[i].ring;  // 获取点的环号
          double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;  // 计算偏航角

          if (is_first[layer])  // 如果是该层的第一个点
          {
            // printf("layer: %d; is first: %d", layer, is_first[layer]);
              yaw_fp[layer]=yaw_angle;  // 设置该层的第一个点的偏航角
              is_first[layer]=false;  // 设置该层的第一个点标志为假
              added_pt.curvature = 0.0;  // 设置曲率为0
              yaw_last[layer]=yaw_angle;  // 设置该层的最后一个点的偏航角
              time_last[layer]=added_pt.curvature;  // 设置该层的最后一个点的时间偏移
              continue;  // 继续下一个点
          }

          // 计算时间偏移
          if (yaw_angle <= yaw_fp[layer])
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle) / omega_l;  // 计算时间偏移
          }
          else
          {
            added_pt.curvature = (yaw_fp[layer]-yaw_angle+360.0) / omega_l;  // 计算时间偏移
          }

          if (added_pt.curvature < time_last[layer])  added_pt.curvature+=360.0/omega_l;  // 如果时间偏移小于最后一个点的时间偏移，调整时间偏移

          yaw_last[layer] = yaw_angle;  // 更新最后一个点的偏航角
          time_last[layer]=added_pt.curvature;  // 更新最后一个点的时间偏移
        }

        if (i % point_filter_num == 0)  // 如果点满足过滤条件
        {
          if(added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > blind)  // 如果点不在盲区内
          {
            pl_surf.points.push_back(added_pt);  // 将点添加到表面点云
          }
        }
      }
    }
}

void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types)
{
  int plsize = pl.size();  // 获取点云大小
  int plsize2;
  if(plsize == 0)  // 如果点云大小为0
  {
    printf("something wrong\n");  // 打印错误信息
    return;  // 返回
  }
  uint head = 0;  // 初始化头部索引

  while(types[head].range < blind)  // 跳过盲区内的点
  {
    head++;  // 增加头部索引
  }

  // Surf
  plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;  // 计算有效点云大小

  Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());  // 当前方向向量
  Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());  // 上一次方向向量

  uint i_nex = 0, i2;  // 初始化下一个索引和临时索引
  uint last_i = 0; uint last_i_nex = 0;  // 初始化上一次索引和上一次下一个索引
  int last_state = 0;  // 初始化上一次状态
  int plane_type;  // 平面类型

  for(uint i=head; i<plsize2; i++)  // 遍历有效点云
  {
    if(types[i].range < blind)  // 如果点在盲区内
    {
      continue;  // 跳过
    }

    i2 = i;  // 设置临时索引

    plane_type = plane_judge(pl, types, i, i_nex, curr_direct);  // 判断平面类型
    
    if(plane_type == 1)  // 如果是实平面
    {
      for(uint j=i; j<=i_nex; j++)  // 遍历平面内的点
      { 
        if(j!=i && j!=i_nex)  // 如果不是平面边界点
        {
          types[j].ftype = Real_Plane;  // 设置为实平面
        }
        else
        {
          types[j].ftype = Poss_Plane;  // 设置为可能平面
        }
      }
      
      // if(last_state==1 && fabs(last_direct.sum())>0.5)
      if(last_state==1 && last_direct.norm()>0.1)  // 如果上一次状态为实平面且方向向量不为零
      {
        double mod = last_direct.transpose() * curr_direct;  // 计算方向向量的点积
        if(mod>-0.707 && mod<0.707)  // 如果点积在一定范围内
        {
          types[i].ftype = Edge_Plane;  // 设置为边缘平面
        }
        else
        {
          types[i].ftype = Real_Plane;  // 设置为实平面
        }
      }
      
      i = i_nex - 1;  // 更新索引
      last_state = 1;  // 更新状态
    }
    else // if(plane_type == 2)  // 如果是其他类型
    {
      i = i_nex;  // 更新索引
      last_state = 0;  // 更新状态
    }
    // else if(plane_type == 0)
    // {
    //   if(last_state == 1)
    //   {
    //     uint i_nex_tem;
    //     uint j;
    //     for(j=last_i+1; j<=last_i_nex; j++)
    //     {
    //       uint i_nex_tem2 = i_nex_tem;
    //       Eigen::Vector3d curr_direct2;

    //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

    //       if(ttem != 1)
    //       {
    //         i_nex_tem = i_nex_tem2;
    //         break;
    //       }
    //       curr_direct = curr_direct2;
    //     }

    //     if(j == last_i+1)
    //     {
    //       last_state = 0;
    //     }
    //     else
    //     {
    //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
    //       {
    //         if(k != i_nex_tem)
    //         {
    //           types[k].ftype = Real_Plane;
    //         }
    //         else
    //         {
    //           types[k].ftype = Poss_Plane;
    //         }
    //       }
    //       i = i_nex_tem-1;
    //       i_nex = i_nex_tem;
    //       i2 = j-1;
    //       last_state = 1;
    //     }

    //   }
    // }

    last_i = i2;  // 更新上一次索引
    last_i_nex = i_nex;  // 更新上一次下一个索引
    last_direct = curr_direct;  // 更新上一次方向向量
  }

  plsize2 = plsize-1;  // 计算有效点云大小
  double ratio;  // 定义比例变量
  for(uint i=head+1; i<plsize2; i++)  // 遍历有效点云
  {
    if(types[i].range<blind || types[i-1].range<blind || types[i+1].range<blind)  // 如果当前点或相邻点在盲区内
    {
      continue;  // 跳过
    }
    
    if(types[i-1].dista<1e-8 || types[i].dista<1e-8)  // 如果相邻点的距离平方接近于零
    {
      continue;  // 跳过
    }

    if(types[i].ftype == Nor)  // 如果当前点的类型为普通点
    {
      if(types[i-1].dista > types[i].dista)  // 如果前一个点的距离平方大于当前点
      {
        ratio = types[i-1].dista / types[i].dista;  // 计算比例
      }
      else
      {
        ratio = types[i].dista / types[i-1].dista;  // 计算比例
      }

      if(types[i].intersect<smallp_intersect && ratio < smallp_ratio)  // 如果交点小于阈值且比例小于阈值
      {
        if(types[i-1].ftype == Nor)  // 如果前一个点的类型为普通点
        {
          types[i-1].ftype = Real_Plane;  // 设置为实平面
        }
        if(types[i+1].ftype == Nor)  // 如果后一个点的类型为普通点
        {
          types[i+1].ftype = Real_Plane;  // 设置为实平面
        }
        types[i].ftype = Real_Plane;  // 设置为实平面
      }
    }
  }

  int last_surface = -1;  // 初始化最后一个表面索引
  for(uint j=head; j<plsize; j++)  // 遍历有效点云
  {
    if(types[j].ftype==Poss_Plane || types[j].ftype==Real_Plane)  // 如果点的类型为可能平面或实平面
    {
      if(last_surface == -1)  // 如果没有记录最后一个表面
      {
        last_surface = j;  // 记录当前索引为最后一个表面
      }
    
      if(j == uint(last_surface+point_filter_num-1))  // 如果当前索引为最后一个表面加上过滤数量
      {
        PointType ap;  // 定义点类型
        ap.x = pl[j].x;  // 设置点的x坐标
        ap.y = pl[j].y;  // 设置点的y坐标
        ap.z = pl[j].z;  // 设置点的z坐标
        ap.intensity = pl[j].intensity;  // 设置点的反射强度
        ap.curvature = pl[j].curvature;  // 设置点的曲率
        pl_surf.push_back(ap);  // 将点添加到表面点云

        last_surface = -1;  // 重置最后一个表面索引
      }
    }
    else  // 如果点的类型不是可能平面或实平面
    {
      if(types[j].ftype==Edge_Jump || types[j].ftype==Edge_Plane)  // 如果点的类型为边缘跳跃或边缘平面
      {
        pl_corn.push_back(pl[j]);  // 将点添加到角点云
      }
      if(last_surface != -1)  // 如果记录了最后一个表面
      {
        PointType ap;  // 定义点类型
        for(uint k=last_surface; k<j; k++)  // 遍历表面点
        {
          ap.x += pl[k].x;  // 累加x坐标
          ap.y += pl[k].y;  // 累加y坐标
          ap.z += pl[k].z;  // 累加z坐标
          ap.intensity += pl[k].intensity;  // 累加反射强度
          ap.curvature += pl[k].curvature;  // 累加曲率
        }
        ap.x /= (j-last_surface);  // 计算平均x坐标
        ap.y /= (j-last_surface);  // 计算平均y坐标
        ap.z /= (j-last_surface);  // 计算平均z坐标
        ap.intensity /= (j-last_surface);  // 计算平均反射强度
        ap.curvature /= (j-last_surface);  // 计算平均曲率
        pl_surf.push_back(ap);  // 将点添加到表面点云
      }
      last_surface = -1;  // 重置最后一个表面索引
    }
  }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct)
{
  pl.height = 1; pl.width = pl.size();  // 设置点云的高度和宽度
  sensor_msgs::PointCloud2 output;  // 定义ROS点云消息
  pcl::toROSMsg(pl, output);  // 将PCL点云转换为ROS消息
  output.header.frame_id = "livox";  // 设置消息的坐标系
  output.header.stamp = ct;  // 设置消息的时间戳
}

int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct)
{
  double group_dis = disA*types[i_cur].range + disB;  // 计算组距离
  group_dis = group_dis * group_dis;  // 平方化组距离

  double two_dis;  // 定义两点距离
  vector<double> disarr;  // 定义距离数组
  disarr.reserve(20);  // 预留数组空间

  for(i_nex=i_cur; i_nex<i_cur+group_size; i_nex++)  // 遍历组内点
  {
    if(types[i_nex].range < blind)  // 如果点在盲区内
    {
      curr_direct.setZero();  // 将当前方向向量置零
      return 2;  // 返回2表示盲区
    }
    disarr.push_back(types[i_nex].dista);  // 将距离平方添加到数组
  }
  
  for(;;)  // 无限循环
  {
    if((i_cur >= pl.size()) || (i_nex >= pl.size())) break;  // 如果索引超出范围，退出循环

    if(types[i_nex].range < blind)  // 如果点在盲区内
    {
      curr_direct.setZero();  // 将当前方向向量置零
      return 2;  // 返回2表示盲区
    }
    vx = pl[i_nex].x - pl[i_cur].x;  // 计算x方向的差值
    vy = pl[i_nex].y - pl[i_cur].y;  // 计算y方向的差值
    vz = pl[i_nex].z - pl[i_cur].z;  // 计算z方向的差值
    two_dis = vx*vx + vy*vy + vz*vz;  // 计算两点距离平方
    if(two_dis >= group_dis)  // 如果两点距离大于等于组距离
    {
      break;  // 退出循环
    }
    disarr.push_back(types[i_nex].dista);  // 将距离平方添加到数组
    i_nex++;  // 增加下一个索引
  }

  double leng_wid = 0;  // 定义长度宽度
  double v1[3], v2[3];  // 定义向量
  for(uint j=i_cur+1; j<i_nex; j++)  // 遍历组内点
  {
    if((j >= pl.size()) || (i_cur >= pl.size())) break;  // 如果索引超出范围，退出循环
    v1[0] = pl[j].x - pl[i_cur].x;  // 计算向量v1的x分量
    v1[1] = pl[j].y - pl[i_cur].y;  // 计算向量v1的y分量
    v1[2] = pl[j].z - pl[i_cur].z;  // 计算向量v1的z分量

    v2[0] = v1[1]*vz - vy*v1[2];  // 计算向量v2的x分量
    v2[1] = v1[2]*vx - v1[0]*vz;  // 计算向量v2的y分量
    v2[2] = v1[0]*vy - vx*v1[1];  // 计算向量v2的z分量

    double lw = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];  // 计算向量v2的长度平方
    if(lw > leng_wid)  // 如果长度平方大于当前最大值
    {
      leng_wid = lw;  // 更新最大值
    }
  }

  if((two_dis*two_dis/leng_wid) < p2l_ratio)  // 如果两点距离平方与长度宽度的比值小于阈值
  {
    curr_direct.setZero();  // 将当前方向向量置零
    return 0;  // 返回0表示非平面
  }

  uint disarrsize = disarr.size();  // 获取距离数组大小
  for(uint j=0; j<disarrsize-1; j++)  // 冒泡排序
  {
    for(uint k=j+1; k<disarrsize; k++)
    {
      if(disarr[j] < disarr[k])  // 如果前一个距离小于后一个
      {
        leng_wid = disarr[j];  // 交换两者
        disarr[j] = disarr[k];
        disarr[k] = leng_wid;
      }
    }
  }

  if(disarr[disarr.size()-2] < 1e-16)  // 如果倒数第二个距离小于阈值
  {
    curr_direct.setZero();  // 将当前方向向量置零
    return 0;  // 返回0表示非平面
  }

  if(lidar_type==AVIA)  // 如果雷达类型为AVIA
  {
    double dismax_mid = disarr[0]/disarr[disarrsize/2];  // 计算最大中间距离比
    double dismid_min = disarr[disarrsize/2]/disarr[disarrsize-2];  // 计算中间最小距离比

    if(dismax_mid>=limit_maxmid || dismid_min>=limit_midmin)  // 如果比值大于等于阈值
    {
      curr_direct.setZero();  // 将当前方向向量置零
      return 0;  // 返回0表示非平面
    }
  }
  else  // 如果雷达类型不是AVIA
  {
    double dismax_min = disarr[0] / disarr[disarrsize-2];  // 计算最大最小距离比
    if(dismax_min >= limit_maxmin)  // 如果比值大于等于阈值
    {
      curr_direct.setZero();  // 将当前方向向量置零
      return 0;  // 返回0表示非平面
    }
  }
  
  curr_direct << vx, vy, vz;  // 设置当前方向向量
  curr_direct.normalize();  // 归一化方向向量
  return 1;  // 返回1表示平面
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir)
{
  if(nor_dir == 0)  // 如果方向为0
  {
    if(types[i-1].range<blind || types[i-2].range<blind)  // 如果前两个点在盲区内
    {
      return false;  // 返回false表示不是边缘跳跃
    }
  }
  else if(nor_dir == 1)  // 如果方向为1
  {
    if(types[i+1].range<blind || types[i+2].range<blind)  // 如果后两个点在盲区内
    {
      return false;  // 返回false表示不是边缘跳跃
    }
  }
  double d1 = types[i+nor_dir-1].dista;  // 获取第一个距离平方
  double d2 = types[i+3*nor_dir-2].dista;  // 获取第二个距离平方
  double d;

  if(d1<d2)  // 如果第一个距离小于第二个
  {
    d = d1;  // 交换两者
    d1 = d2;
    d2 = d;
  }

  d1 = sqrt(d1);  // 计算第一个距离
  d2 = sqrt(d2);  // 计算第二个距离

  if(d1>edgea*d2 || (d1-d2)>edgeb)  // 如果第一个距离大于第二个距离的倍数或两者差值大于阈值
  {
    return false;  // 返回false表示不是边缘跳跃
  }
  
  return true;  // 返回true表示是边缘跳跃
}

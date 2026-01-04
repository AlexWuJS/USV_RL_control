#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Vector3.h>
#include "use-ikfom.hpp"

/// *************Preconfiguration

#define MAX_INI_COUNT (10)

const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};  // 时间排序

/// *************IMU Process and undistortion
class ImuProcess  // 惯性测量单元处理类
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保Eigen的内存对齐

  ImuProcess();  // 构造函数
  ~ImuProcess(); // 析构函数
  
  void Reset();  // 重置
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);  // 重置
  void set_extrinsic(const V3D &transl, const M3D &rot);  // 设置外参
  void set_extrinsic(const V3D &transl);  // 设置外参
  void set_extrinsic(const MD(4,4) &T);  // 设置外参
  void set_gyr_cov(const V3D &scaler);  // 设置陀螺仪噪声
  void set_acc_cov(const V3D &scaler);  // 设置加速度计噪声
  void set_gyr_bias_cov(const V3D &b_g);  // 设置陀螺仪偏置噪声
  void set_acc_bias_cov(const V3D &b_a);  // 设置加速度计偏置噪声
  Eigen::Matrix<double, 12, 12> Q;  // 状态协方差矩阵
  void Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  ofstream fout_imu;  // 输出IMU数据
  V3D cov_acc;  // 加速度计协方差
  V3D cov_gyr;  // 陀螺仪协方差
  V3D cov_acc_scale;  // 加速度计缩放
  V3D cov_gyr_scale;  // 陀螺仪缩放
  V3D cov_bias_gyr;  // 陀螺仪偏置协方差
  V3D cov_bias_acc;  // 加速度计偏置协方差
  double first_lidar_time;  // 第一个激光雷达时间

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);  // 初始化IMU
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);  // 去畸变点云

  PointCloudXYZI::Ptr cur_pcl_un_;  // 当前未畸变点云
  sensor_msgs::ImuConstPtr last_imu_;  // 最后一个IMU数据
  deque<sensor_msgs::ImuConstPtr> v_imu_;  // IMU数据队列
  vector<Pose6D> IMUpose;  // IMU位姿
  vector<M3D>    v_rot_pcl_;  // 点云旋转
  M3D Lidar_R_wrt_IMU;  // 雷达相对于IMU的旋转
  V3D Lidar_T_wrt_IMU;  // 雷达相对于IMU的平移
  V3D mean_acc;  // 平均加速度
  V3D mean_gyr;  // 平均角速度
  V3D angvel_last;  // 角速度
  V3D acc_s_last;  // 加速度
  double start_timestamp_;  // 开始时间
  double last_lidar_end_time_;  // 最后一个激光雷达时间
  int    init_iter_num = 1;  // 初始化迭代次数
  bool   b_first_frame_ = true;  // 是否是第一个激光雷达帧
  bool   imu_need_init_ = true;  // 是否需要初始化
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), start_timestamp_(-1)  // 初始化
{
  init_iter_num = 1;  // 初始化迭代次数
  Q = process_noise_cov();  // 过程噪声协方差
  cov_acc       = V3D(0.1, 0.1, 0.1);  // 加速度计协方差
  cov_gyr       = V3D(0.1, 0.1, 0.1);  // 陀螺仪协方差
  cov_bias_gyr  = V3D(0.0001, 0.0001, 0.0001);  // 陀螺仪偏置协方差
  cov_bias_acc  = V3D(0.0001, 0.0001, 0.0001);  // 加速度计偏置协方差   
  mean_acc      = V3D(0, 0, -1.0);  // 平均加速度
  mean_gyr      = V3D(0, 0, 0);  // 平均角速度
  angvel_last     = Zero3d;  // 角速度
  Lidar_T_wrt_IMU = Zero3d;  // 雷达相对于IMU的平移
  Lidar_R_wrt_IMU = Eye3d;  // 雷达相对于IMU的旋转
  last_imu_.reset(new sensor_msgs::Imu());  // 最后一个IMU数据
}

ImuProcess::~ImuProcess() {}  // 析构函数

void ImuProcess::Reset()  // 重置
{
  // ROS_WARN("Reset ImuProcess");
  mean_acc      = V3D(0, 0, -1.0);  // 平均加速度
  mean_gyr      = V3D(0, 0, 0);  // 平均角速度
  angvel_last       = Zero3d;  // 角速度
  imu_need_init_    = true;  // 是否需要初始化
  start_timestamp_  = -1;  // 开始时间
  init_iter_num     = 1;  // 初始化迭代次数
  v_imu_.clear();  // 清空IMU数据队列
  IMUpose.clear();  // 清空IMU位姿
  last_imu_.reset(new sensor_msgs::Imu());  // 最后一个IMU数据
  cur_pcl_un_.reset(new PointCloudXYZI());  // 当前未畸变点云
}

void ImuProcess::set_extrinsic(const MD(4,4) &T)  // 设置外参
{
  Lidar_T_wrt_IMU = T.block<3,1>(0,3);  // 雷达相对于IMU的平移
  Lidar_R_wrt_IMU = T.block<3,3>(0,0);  // 雷达相对于IMU的旋转
}

void ImuProcess::set_extrinsic(const V3D &transl)  // 设置外参
{
  Lidar_T_wrt_IMU = transl;  // 雷达相对于IMU的平移
  Lidar_R_wrt_IMU.setIdentity();  // 雷达相对于IMU的旋转
}

void ImuProcess::set_extrinsic(const V3D &transl, const M3D &rot)  // 设置外参
{
  Lidar_T_wrt_IMU = transl;  // 雷达相对于IMU的平移
  Lidar_R_wrt_IMU = rot;  // 雷达相对于IMU的旋转
}

void ImuProcess::set_gyr_cov(const V3D &scaler)  // 设置陀螺仪协方差
{
  cov_gyr_scale = scaler;  // 陀螺仪缩放
}

void ImuProcess::set_acc_cov(const V3D &scaler)  // 设置加速度计协方差
{
  cov_acc_scale = scaler;  // 加速度计缩放  
}

void ImuProcess::set_gyr_bias_cov(const V3D &b_g)  // 设置陀螺仪偏置协方差
{
  cov_bias_gyr = b_g;  // 陀螺仪偏置协方差
}

void ImuProcess::set_acc_bias_cov(const V3D &b_a)  // 设置加速度计偏置协方差
{
  cov_bias_acc = b_a;  // 加速度计偏置协方差
}

void ImuProcess::IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N)  // 初始化IMU 
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  
  V3D cur_acc, cur_gyr;  // 当前加速度和角速度
  
  if (b_first_frame_)  // 如果是第一个激光雷达帧
  {
    Reset();  // 重置
    N = 1;  // 初始化迭代次数
    b_first_frame_ = false;  // 是否是第一个激光雷达帧
    const auto &imu_acc = meas.imu.front()->linear_acceleration;  // 激光雷达帧的加速度
    const auto &gyr_acc = meas.imu.front()->angular_velocity;  // 激光雷达帧的角速度
    mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;  // 平均加速度
    mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;  // 平均角速度
    first_lidar_time = meas.lidar_beg_time;  // 第一个激光雷达时间
  }

  for (const auto &imu : meas.imu)  // 遍历IMU数据
  {
    const auto &imu_acc = imu->linear_acceleration;  // 当前IMU的加速度
    const auto &gyr_acc = imu->angular_velocity;  // 当前IMU的角速度
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;  // 当前加速度
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    mean_acc      += (cur_acc - mean_acc) / N;  // 更新平均加速度
    mean_gyr      += (cur_gyr - mean_gyr) / N;  // 更新平均角速度

    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);  // 更新加速度协方差
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);  // 更新角速度协方差

    // cout<<"acc norm: "<<cur_acc.norm()<<" "<<mean_acc.norm()<<endl;

    N ++;  // 更新迭代次数
  }
  state_ikfom init_state = kf_state.get_x();  // 初始化状态
  init_state.grav = S2(- mean_acc / mean_acc.norm() * G_m_s2);  // 重力加速度
  
  //state_inout.rot = Eye3d; // Exp(mean_acc.cross(V3D(0, 0, -1 / scale_gravity)));
  init_state.bg  = mean_gyr;  // 陀螺仪偏置
  init_state.offset_T_L_I = Lidar_T_wrt_IMU;  // 雷达相对于IMU的平移
  init_state.offset_R_L_I = Lidar_R_wrt_IMU;  // 雷达相对于IMU的旋转
  kf_state.change_x(init_state);  // 更新状态

  esekfom::esekf<state_ikfom, 12, input_ikfom>::cov init_P = kf_state.get_P();  // 初始化协方差
  init_P.setIdentity();  // 设置为单位矩阵
  init_P(6,6) = init_P(7,7) = init_P(8,8) = 0.00001;  // 设置加速度计偏置协方差
  init_P(9,9) = init_P(10,10) = init_P(11,11) = 0.00001;  // 设置陀螺仪偏置协方差
  init_P(15,15) = init_P(16,16) = init_P(17,17) = 0.0001;  // 设置加速度计协方差
  init_P(18,18) = init_P(19,19) = init_P(20,20) = 0.001;  // 设置陀螺仪协方差
  init_P(21,21) = init_P(22,22) = 0.00001;  // 设置陀螺仪偏置协方差 
  kf_state.change_P(init_P);  // 更新协方差
  last_imu_ = meas.imu.back();  // 最后一个IMU数据

}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_out)   // 去畸变点云
{
  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  const double &pcl_end_time = meas.lidar_end_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  // cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
  //          <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<endl;

  /*** Initialize IMU pose ***/
  state_ikfom imu_state = kf_state.get_x();  // 获取当前状态
  IMUpose.clear();  // 清空IMU位姿
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));  // 添加IMU位姿

  /*** forward propagation at each imu point ***/
  V3D angvel_avr, acc_avr, acc_imu, vel_imu, pos_imu;  // 角速度平均值，加速度平均值，加速度，速度，位置
  M3D R_imu;  // 旋转矩阵

  double dt = 0;  // 时间差

  input_ikfom in;  // 输入
  for (auto it_imu = v_imu.begin(); it_imu < (v_imu.end() - 1); it_imu++)  // 遍历IMU数据
  {
    auto &&head = *(it_imu);  // 当前IMU数据
    auto &&tail = *(it_imu + 1);  // 下一个IMU数据
    
    if (tail->header.stamp.toSec() < last_lidar_end_time_)    continue;  // 如果IMU数据结束时间小于激光雷达结束时间，则跳过
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),  // 角速度平均值
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);  // 角速度平均值 
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),  // 加速度平均值
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);  // 加速度平均值

    // fout_imu << setw(10) << head->header.stamp.toSec() - first_lidar_time << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << endl;

    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm(); // - state_inout.ba;  // 加速度归一化

    if(head->header.stamp.toSec() < last_lidar_end_time_)  // 如果IMU数据开始时间小于激光雷达结束时间
    {
      dt = tail->header.stamp.toSec() - last_lidar_end_time_;  // 时间差
      // dt = tail->header.stamp.toSec() - pcl_beg_time;
    }
    else
    {
      dt = tail->header.stamp.toSec() - head->header.stamp.toSec();  // 时间差
    }
    
    in.acc = acc_avr;  // 加速度
    in.gyro = angvel_avr;  // 角速度
    Q.block<3, 3>(0, 0).diagonal() = cov_gyr;  // 陀螺仪协方差
    Q.block<3, 3>(3, 3).diagonal() = cov_acc;  // 加速度计协方差
    Q.block<3, 3>(6, 6).diagonal() = cov_bias_gyr;  // 陀螺仪偏置协方差
    Q.block<3, 3>(9, 9).diagonal() = cov_bias_acc;  // 加速度计偏置协方差
    kf_state.predict(dt, Q, in);  // 预测

    /* save the poses at each IMU measurements */
    imu_state = kf_state.get_x();  // 获取当前状态
    angvel_last = angvel_avr - imu_state.bg;  // 角速度
    acc_s_last  = imu_state.rot * (acc_avr - imu_state.ba);  // 加速度
    for(int i=0; i<3; i++)
    {
      acc_s_last[i] += imu_state.grav[i];  // 重力加速度    
    }
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;  // 时间差
    IMUpose.push_back(set_pose6d(offs_t, acc_s_last, angvel_last, imu_state.vel, imu_state.pos, imu_state.rot.toRotationMatrix()));  // 添加IMU位姿
  } 

  /*** calculated the pos and attitude prediction at the frame-end ***/
  double note = pcl_end_time > imu_end_time ? 1.0 : -1.0;  // 时间差
  dt = note * (pcl_end_time - imu_end_time);  // 时间差
  kf_state.predict(dt, Q, in);  // 预测
  
  imu_state = kf_state.get_x();  // 获取当前状态
  last_imu_ = meas.imu.back();  // 最后一个IMU数据
  last_lidar_end_time_ = pcl_end_time;  // 激光雷达结束时间

  /*** undistort each lidar point (backward propagation) ***/
  if (pcl_out.points.begin() == pcl_out.points.end()) return;
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;  // 当前IMU位姿
    auto tail = it_kp;  // 下一个IMU位姿
    R_imu<<MAT_FROM_ARRAY(head->rot);  // 旋转矩阵
    // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);  // 速度
    pos_imu<<VEC_FROM_ARRAY(head->pos);  // 位置
    acc_imu<<VEC_FROM_ARRAY(tail->acc);  // 加速度
    angvel_avr<<VEC_FROM_ARRAY(tail->gyr);  // 角速度

    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)  // 遍历激光雷达点云
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;  // 时间差

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      M3D R_i(R_imu * Exp(angvel_avr, dt));  // 旋转矩阵
      
      V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);  // 点云位置
      V3D T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt - imu_state.pos);  // 位置
      V3D P_compensate = imu_state.offset_R_L_I.conjugate() * (imu_state.rot.conjugate() * (R_i * (imu_state.offset_R_L_I * P_i + imu_state.offset_T_L_I) + T_ei) - imu_state.offset_T_L_I);// not accurate!  // 补偿点云位置
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);  // 点云位置
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin()) break;  // 如果激光雷达点云结束时间小于IMU位姿开始时间，则跳出
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas,  esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr cur_pcl_un_)  // 处理IMU数据
{
  double t1,t2,t3;  // 时间
  t1 = omp_get_wtime();  // 时间

  if(meas.imu.empty()) {return;};  // 如果IMU数据为空，则返回
  ROS_ASSERT(meas.lidar != nullptr);  // 断言激光雷达数据不为空

  if (imu_need_init_)  // 如果IMU需要初始化
  {
    /// The very first lidar frame
    IMU_init(meas, kf_state, init_iter_num);  // 初始化IMU

    imu_need_init_ = true;  // 设置IMU需要初始化
    
    last_imu_   = meas.imu.back();  // 最后一个IMU数据

    state_ikfom imu_state = kf_state.get_x();  // 获取当前状态
    if (init_iter_num > MAX_INI_COUNT)  // 如果迭代次数大于最大迭代次数
    {
      cov_acc *= pow(G_m_s2 / mean_acc.norm(), 2);  // 加速度协方差
      imu_need_init_ = false;  // 设置IMU不需要初始化

      cov_acc = cov_acc_scale;  // 加速度协方差
      cov_gyr = cov_gyr_scale;  // 角速度协方差
      ROS_INFO("IMU Initial Done");  // 初始化完成
      // ROS_INFO("IMU Initial Done: Gravity: %.4f %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",\
      //          imu_state.grav[0], imu_state.grav[1], imu_state.grav[2], mean_acc.norm(), cov_bias_gyr[0], cov_bias_gyr[1], cov_bias_gyr[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
      fout_imu.open(DEBUG_FILE_DIR("imu.txt"),ios::out);  // 打开IMU文件
    }

    return;
  }

  UndistortPcl(meas, kf_state, *cur_pcl_un_);  // 去畸变点云

  t2 = omp_get_wtime();  // 时间
  t3 = omp_get_wtime();  // 时间
  
  // cout<<"[ IMU Process ]: Time: "<<t3 - t1<<endl;
}

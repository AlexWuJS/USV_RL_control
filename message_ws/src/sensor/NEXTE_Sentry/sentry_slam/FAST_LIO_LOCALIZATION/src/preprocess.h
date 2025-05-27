#include <ros/ros.h>  // 包含ROS的基本功能库
#include <pcl_conversions/pcl_conversions.h>  // 包含PCL与ROS消息转换的库
#include <sensor_msgs/PointCloud2.h>  // 包含ROS点云消息类型
#include <livox_ros_driver2/CustomMsg.h>  // 包含Livox自定义消息类型

using namespace std;  // 使用标准命名空间

#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)  // 定义一个宏，用于判断一个值是否有效

typedef pcl::PointXYZINormal PointType;  // 定义点类型为PCL的XYZI法线点
typedef pcl::PointCloud<PointType> PointCloudXYZI;  // 定义点云类型为PCL的点云

enum LID_TYPE{AVIA = 1, VELO16, OUST64}; // 定义激光雷达类型枚举
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};  // 定义特征类型枚举
enum Surround{Prev, Next};  // 定义周围点枚举
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};  // 定义跳跃类型枚举

struct orgtype  // 定义一个结构体用于存储点的原始信息
{
  double range;  // 点的距离
  double dista;  // 点的距离平方
  double angle[2];  // 点的角度
  double intersect;  // 点的交点
  E_jump edj[2];  // 点的跳跃类型
  Feature ftype;  // 点的特征类型
  orgtype()  // 结构体的构造函数
  {
    range = 0;  // 初始化距离为0
    edj[Prev] = Nr_nor;  // 初始化前一个跳跃类型为正常
    edj[Next] = Nr_nor;  // 初始化下一个跳跃类型为正常
    ftype = Nor;  // 初始化特征类型为普通
    intersect = 2;  // 初始化交点为2
  }
};

namespace velodyne_ros {  // 定义一个命名空间用于Velodyne点云
  struct EIGEN_ALIGN16 Point {  // 定义一个结构体用于存储点信息
      PCL_ADD_POINT4D;  // 添加4D点信息
      float intensity;  // 点的强度
      float time;  // 点的时间
      uint16_t ring;  // 点的环号
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,  // 注册Velodyne点云结构
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {  // 定义一个命名空间用于Ouster点云
  struct EIGEN_ALIGN16 Point {  // 定义一个结构体用于存储点信息
      PCL_ADD_POINT4D;  // 添加4D点信息
      float intensity;  // 点的强度
      uint32_t t;  // 点的时间
      uint16_t reflectivity;  // 点的反射率
      uint8_t  ring;  // 点的环号
      uint16_t ambient;  // 点的环境光
      uint32_t range;  // 点的距离
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,  // 注册Ouster点云结构
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // 使用std::uint32_t以避免与pcl::uint32_t冲突
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess  // 定义一个预处理类
{
  public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // 确保内存对齐

  Preprocess();  // 构造函数
  ~Preprocess();  // 析构函数
  
  void process(const livox_ros_driver2::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);  // 处理Livox消息
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);  // 处理ROS点云消息
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);  // 设置预处理参数

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;  // 点云消息指针
  PointCloudXYZI pl_full, pl_corn, pl_surf;  // 定义完整、角点和表面点云
  PointCloudXYZI pl_buff[128]; // 定义点云缓冲区，最大支持128线激光雷达
  vector<orgtype> typess[128]; // 定义点类型缓冲区，最大支持128线激光雷达
  int lidar_type, point_filter_num, N_SCANS;  // 定义激光雷达类型、点过滤数量和扫描线数
  double blind;  // 定义盲区距离
  double max_scan_range;  // 定义最大扫描距离
  bool feature_enabled, given_offset_time;  // 定义特征提取开关和时间偏移标志
  ros::Publisher pub_full, pub_surf, pub_corn;  // 定义ROS发布者，用于发布完整、表面和角点云

  private:
  void avia_handler(const livox_ros_driver2::CustomMsg::ConstPtr &msg);  // 处理Livox消息
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);  // 处理Ouster消息
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);  // 处理Velodyne消息
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);  // 提取特征
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);  // 发布点云
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);  // 判断平面
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);  // 判断小平面
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);  // 判断边缘跳跃
  
  int group_size;  // 定义组大小
  double disA, disB, inf_bound;  // 定义距离参数和无穷边界
  double limit_maxmid, limit_midmin, limit_maxmin;  // 定义限制参数
  double p2l_ratio;  // 定义平面到线的比率
  double jump_up_limit, jump_down_limit;  // 定义跳跃限制
  double cos160;  // 定义余弦值
  double edgea, edgeb;  // 定义边缘参数
  double smallp_intersect, smallp_ratio;  // 定义小平面参数
  double vx, vy, vz;  // 定义向量分量
};

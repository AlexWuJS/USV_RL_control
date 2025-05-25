#include "path/path.h"

Path::Path(tf2_ros::Buffer& tf):
    tf_(tf), has_start_(false), has_goal_(false)
{
    //创建当前状态保存
    current_state_.resize(6);
    
    cout << "初始化 costmap" << endl;
    local_costmap_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    costmap_ = local_costmap_->getCostmap();

    global_frame_ = local_costmap_->getGlobalFrameID();
    robot_base_frame_ = local_costmap_->getBaseFrameID();

    start_sub_ = nh_.subscribe("/initialpose", 1, &Path::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &Path::goalCallback, this);
    global_path_vis_pub_ = nh.advertise<nav_msgs::Path>("/global_path",1);

    left_thrust_pub = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/left_thrust_cmd", 1);
    right_thrust_pub = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/right_thrust_cmd", 1);
    mid_thrust_pub = nh_.advertise<std_msgs::Float32>("/wamv/thrusters/lateral_thrust_cmd", 1);
    predict_path_pub = nh_.advertise<nav_msgs::Path>("/predict_path", 1);
    motion_path_pub = nh_.advertise<nav_msgs::Path>("/motion_path", 1);

    odom_sub_ = nh_.subscribe("/wamv/sensors/position/p3d_wamv", 1, &Path::odomCallback, this);

    control_cmd_pub_ = nh.createTimer(ros::Duration(0.01), &HybridAstarTebNode::publish_control_cmd, this);
}

Path::~Path()
{
}

void Path::startCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    start_pose_[0] = msg.pose.pose.position.x;
    start_pose_[1] = msg.pose.pose.position.y;
    start_pose_[2] = tf2::getYaw(msg.pose.pose.orientation);
    // geometry_msgs::PoseStamped p;
    // p.pose = msg.pose.pose;
    start_pose = PoseSE2(msg.pose.pose);
    
    has_start_ = true;
    // cout << "起始点坐标：" << start_pose_[0] << "," << start_pose_[1] << endl;
    ROS_INFO("receive start position");

}

void Path::goalCallback(const geometry_msgs::PoseStamped msg)
{
    goal_pose_[0] = msg.pose.position.x;
    goal_pose_[1] = msg.pose.position.y;
    goal_pose_[2] = tf2::getYaw(msg.pose.orientation);
    goal_pose = PoseSE2(msg.pose);
    
    has_goal_ = true;
    //cout << "终止点坐标：" << goal_pose_[0] << "," << goal_pose_[1] << endl;

    ROS_INFO("receive goal position");
}

void Path::odomCallback(const nav_msgs::Odometry &msg)
{
    odom_ = msg;
    current_state_(0) = msg.pose.pose.position.x;
    current_state_(1) = msg.pose.pose.position.y;
    current_state_(2) = tf2::getYaw(msg.pose.pose.orientation);
    //if (current_state(2) < 0) current_state(2) += 2 * M_PI;
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    Eigen::Vector2d vel_b;
    double psi = current_state_(2);
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << odom_.twist.twist.linear.x, odom_.twist.twist.linear.y;
    vel_b = R_b_n.transpose() * vel_n;
    current_state_(3) = vel_b(0);
    current_state_(4) = vel_b(1);
    current_state_(5) = msg.twist.twist.angular.z;

    motion_path.header.frame_id = "map";
    motion_path.header.stamp = ros::Time::now();
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = msg.header.stamp;
    pose_msg.header.frame_id = msg.header.frame_id;
    pose_msg.pose.position.x = msg.pose.pose.position.x; 
    pose_msg.pose.position.y = msg.pose.pose.position.y; 
    pose_msg.pose.position.z = 0;
    pose_msg.pose.orientation.x = msg.pose.pose.orientation.x; 
    pose_msg.pose.orientation.y = msg.pose.pose.orientation.y; 
    pose_msg.pose.orientation.z = msg.pose.pose.orientation.z; 
    pose_msg.pose.orientation.w = msg.pose.pose.orientation.w; 
    motion_path.poses.push_back(pose_msg);
    motion_path_pub.publish(motion_path);

    has_odom_ = true;

}
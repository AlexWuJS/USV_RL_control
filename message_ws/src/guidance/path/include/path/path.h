#ifndef _PATH_H_
#define _PATH_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>

#include <Eigen/Dense>
#include <string>
#include <vector>

#define STEP    0.5
#define Side_length     50


bool has_start_ = false;
bool has_goal_ = false;
bool has_odom_ = false;

nav_msgs::Path My_path;

Eigen::Vector3d start_pose_, goal_pose_;

// class Path{
//     public:
//         Path(tf2_ros::Buffer& tf);
//         ~Path();

//         bool has_start_, has_goal_;
//         Eigen::Vector3d start_pose_, goal_pose_;

//         std::list<Eigen::Vector3d> global_path_;

//     private:
//         tf2_ros::Buffer& tf_;
//         costmap_2d::Costmap2DROS* local_costmap_;

//         std::string global_frame_;
//         std::string robot_base_frame_;

//         ros::Publisher left_thrust_pub, mid_thrust_pub, right_thrust_pub, motion_path_pub, predict_path_pub;
//         ros::Subscriber start_sub_, goal_sub_, odom_sub_;
//         nav_msgs::Odometry odom_;
//         nav_msgs::Path predict_path, motion_path;

//         Eigen::VectorXd current_state_;

// };


#endif
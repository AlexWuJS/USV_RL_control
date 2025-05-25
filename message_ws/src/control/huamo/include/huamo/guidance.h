#ifndef _GUIDANCE_H_
#define _GUIDANCE_H_

#include <std_msgs/Float32.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include <tf2/utils.h>

#include <Eigen/Dense>
#include <cmath>

extern bool has_path_;
// extern Eigen::VectorXd current_state_;
// extern nav_msgs::Path path_;

void getDesiredSpeedHeading(double &desired_speed, double &desired_heading, const Eigen::VectorXd current_state, nav_msgs::Path &path,float &ye,Eigen::VectorXd &LOS);
void calculat_expect_yaw(double &desired_heading,const Eigen::VectorXd current_state,Eigen::VectorXd &LOS);

#endif
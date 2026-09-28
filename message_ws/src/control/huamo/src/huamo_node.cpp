#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>
#include <memory>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "huamo/guidance.h"
#include "huamo/huamo.h"
#include "huamo/pwm_data.h"
#include <thruster_manager/Thruster.h>

bool data_flag = false;
bool has_odom_ = false;
bool has_imu_ = false;
bool has_path_ = false;
bool remember = false;

static std::ofstream file;

Eigen::VectorXd current_state_(6);

nav_msgs::Path motion_path, path_, LOS_path;
nav_msgs::Odometry odom_;

static ros::Publisher controller_pub;
thruster_manager::Thruster thruster_msg;

ros::Publisher motion_path_pub, global_path_pub,LOS_path_pub;

std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;

void odomCallback(const nav_msgs::Odometry &msg)
{
    odom_ = msg;
    current_state_(0) = msg.pose.pose.position.x;
    current_state_(1) = msg.pose.pose.position.y;
    // current_state_(2) = atan2(-(current_state_(0)-last_x),(current_state_(1)-last_y));
    current_state_(2) = tf2::getYaw(msg.pose.pose.orientation) + 91.5*M_PI/180.f;
    current_state_(2) = current_state_(2) > M_PI ? (current_state_(2) - 2*M_PI) : (current_state_(2) < -M_PI ? (current_state_(2) + 2 * M_PI) : current_state_(2));
    // std::cout << "yaw:" << current_state_(2)*180/M_PI << std::endl;
    // if (current_state(2) < 0) current_state(2) += 2 * M_PI;
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    Eigen::Matrix2d R_b_n;
    Eigen::Vector2d vel_n;
    Eigen::Vector2d vel_b;
    double psi = current_state_(2);
    R_b_n << cos(psi), -sin(psi), sin(psi), cos(psi);
    vel_n << vx, vy;
    vel_b = R_b_n.transpose() * vel_n;
    current_state_(3) = vel_b(0);
    current_state_(4) = vel_b(1);       //横向速度向左为正
    current_state_(5) = msg.twist.twist.angular.z;

    // std::cout << "u" << vel_b(0) << std::endl;

    // 只在任务执行期间累积行驶轨迹：任务结束/取消后不再增长，
    // 否则空转时里程计会不断往同一位置堆点，消息无限膨胀
    if (has_path_) {
        motion_path.header.frame_id = "camera_init";
        motion_path.header.stamp = ros::Time::now();
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = msg.header.stamp;
        pose_msg.header.frame_id = "camera_init";
        pose_msg.pose.position.x = msg.pose.pose.position.x; 
        pose_msg.pose.position.y = msg.pose.pose.position.y; 
        pose_msg.pose.position.z = 0;
        pose_msg.pose.orientation.x = msg.pose.pose.orientation.x; 
        pose_msg.pose.orientation.y = msg.pose.pose.orientation.y; 
        pose_msg.pose.orientation.z = msg.pose.pose.orientation.z; 
        pose_msg.pose.orientation.w = msg.pose.pose.orientation.w; 
        motion_path.poses.push_back(pose_msg);
        motion_path_pub.publish(motion_path);
    }

    has_odom_ = true;

}

void pathCallback(const nav_msgs::Path &path)
{
    // 空路径 = 取消任务：清空路径与轨迹，并显式输出中位 PWM 让艇停下。
    // （thruster_manager 会保持最后收到的 PWM，所以取消时必须显式发停车指令）
    if (path.poses.empty()) {
        path_.poses.clear();
        motion_path.poses.clear();
        LOS_path.poses.clear();
        has_path_ = false;
        thruster_manager::Thruster stop_msg;
        stop_msg.left_thruster_pwm = 1500;
        stop_msg.right_thruster_pwm = 1500;
        stop_msg.left_sideThruster_pwm = 1500;
        stop_msg.right_sideThruster_pwm = 1500;
        controller_pub.publish(stop_msg);
        // 把清空后的空轨迹发布出去，否则 RViz 仍显示最后收到的旧轨迹
        motion_path_pub.publish(motion_path);
        LOS_path_pub.publish(LOS_path);
        ROS_INFO("huamo: 收到空路径 -> 取消任务、停车、清空轨迹");
        return;
    }

    if (!tf_buffer_ptr) {
        ROS_WARN_THROTTLE(1.0, "TF buffer not ready, skip path update");
        return;
    }

    const std::string target_frame = "camera_init";
    path_.header = path.header;
    path_.poses.clear();

    for (const auto &pose : path.poses) {
        geometry_msgs::PoseStamped in_pose = pose;
        if (in_pose.header.frame_id.empty()) {
            in_pose.header.frame_id = path.header.frame_id;
        }

        if (in_pose.header.frame_id != target_frame) {
            try {
                geometry_msgs::PoseStamped transformed = tf_buffer_ptr->transform(in_pose, target_frame, ros::Duration(0.1));
                path_.poses.emplace_back(transformed);
            } catch (tf2::TransformException &ex) {
                ROS_WARN_THROTTLE(1.0, "Failed to transform path pose to %s: %s", target_frame.c_str(), ex.what());
            }
        } else {
            path_.poses.emplace_back(in_pose);
        }
    }

    path_.header.frame_id = target_frame;
    path_.header.stamp = path.header.stamp;

    // 新任务到达：清空上一轮的行驶轨迹与前视线轨迹。
    // 否则 /motion_path、/LOS_path 会跨任务无限累积，RViz 里旧轨迹永远留着
    motion_path.poses.clear();
    LOS_path.poses.clear();

    has_path_ = !path_.poses.empty();
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data)
{
    // current_state_(2) = tf::getYaw(imu_data->orientation) - 80*M_PI/180.f;
    // current_state_(2) = current_state_(2) > M_PI ? (current_state_(2) - 2*M_PI) : (current_state_(2) < -M_PI ? (current_state_(2) + 2 * M_PI) : current_state_(2));
    current_state_(5) = imu_data->angular_velocity.z;
    current_state_(2) = current_state_(2) + current_state_(5) * 0.01;
    has_imu_ = true;
}

void huamo_run(const float now_v, const float now_yaw, const float now_w)
{
    if(has_path_)
    {
        static double desired_speed_ = 0, desired_heading_ = 0;
        static float ye_ = 0;
        float forward = 0;
        float turn = 0;
        Eigen::VectorXd LOS_(2);

        // if(has_imu_)
        {
            if(has_odom_)
            {
                getDesiredSpeedHeading(desired_speed_,desired_heading_,current_state_,path_,ye_,LOS_);

                // stop 分支会置 has_path_=false 且不更新 LOS_（局部变量本轮保持零初始化），
                // 此时不得再发布 LOS，否则会推出 (0,0) 野点；
                // 但必须显式发一拍中位 PWM 再静默 —— thruster_manager 会保持最后收到的
                // PWM，直接 return 会让艇带着最后一拍（巡航油门/转弯差速）冲过终点
                // 不停、甚至原地转圈（2026-09-28 水试复现）。
                if (!has_path_)
                {
                    thruster_manager::Thruster stop_msg;
                    stop_msg.left_thruster_pwm = 1500;
                    stop_msg.right_thruster_pwm = 1500;
                    stop_msg.left_sideThruster_pwm = 1500;
                    stop_msg.right_sideThruster_pwm = 1500;
                    controller_pub.publish(stop_msg);
                    ROS_INFO("huamo: 任务结束(到终点/追路停滞)，停车");
                    has_odom_ = false;
                    return;
                }

                LOS_path.header.frame_id = "camera_init";
                LOS_path.header.stamp = ros::Time::now();
                geometry_msgs::PoseStamped pose_msg;
                pose_msg.header.stamp = ros::Time::now();
                pose_msg.header.frame_id = "camera_init";
                pose_msg.pose.position.x = LOS_[0]; 
                pose_msg.pose.position.y = LOS_[1]; 
                pose_msg.pose.position.z = 0;
                LOS_path.poses.push_back(pose_msg);
                LOS_path_pub.publish(LOS_path);

                forward = speed_huamo(now_v,desired_speed_);
                
                turn = yaw_huamo(forward, now_yaw, desired_heading_, now_w,ye_);
                std::vector<float> control_ = remote_data_deal(forward,turn);
                thruster_msg.left_thruster_pwm = control_[0];
                thruster_msg.right_thruster_pwm = control_[1];
                controller_pub.publish(thruster_msg);
                
                has_odom_ = false;
            }
            
        }
        
    }
}

void data_flag_callback(const std_msgs::Bool::ConstPtr& data)
{
    data_flag = data->data;
    if(!data_flag)
    {
        remember = false;
    }
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"huamo_node");

    // ros::Rate r(10);

    ros::NodeHandle nh;

    // 引导律参数：launch 里用 <param name="speed_max" .../> 覆盖（默认 2.0 保持原行为）
    ros::NodeHandle pnh("~");
    pnh.param<float>("speed_max", speed_max, 2.0f);
    ROS_INFO("huamo_node: speed_max = %.2f m/s", speed_max);

    tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);

    // ros::Subscriber odom_sub = nh.subscribe("/odom_local",10,odomCallback);
    // ros::Subscriber imu_sub = nh.subscribe("/imu/data",10,imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odometry",10,odomCallback);
    ros::Subscriber path_sub = nh.subscribe("/my_path",10, pathCallback);
    // ros::Subscriber data_flag_sub = nh.subscribe("data_flag",10,data_flag_callback);

    controller_pub = nh.advertise<thruster_manager::Thruster>("/auto_thruster_pwm", 1000);
    global_path_pub = nh.advertise<nav_msgs::Path>("/global_path",1);
    motion_path_pub = nh.advertise<nav_msgs::Path>("/motion_path", 1);
    LOS_path_pub = nh.advertise<nav_msgs::Path>("/LOS_path", 1);

    // ros::Timer control_cmd = nh.createTimer(ros::Duration(0.005), controler_cmd);
    
    while(ros::ok())
    {
        if(has_odom_)
        {
            static float last_yaw = 0;
            float u = current_state_(3);
            float v = current_state_(4);
            float d_yaw = atan2(v,u);
            float yaw = current_state_(2) + 0*d_yaw;
            // std::cout << "yaw1:" << yaw*180/M_PI << "\tyaw2:" << (current_state_(2)+ 0.6*d_yaw)*180/M_PI << std::endl;
            float w = 10 * (yaw - last_yaw);
            huamo_run(u, yaw, w);

            last_yaw = yaw;
            has_odom_ = false;
        }
        ros::spinOnce();
    }

    ros::spin();

    return 0;
}

#include "ros/ros.h"

#include <cmath>
#include <fstream>
#include <chrono>
#include <ctime>
#include <sstream>

#include "path/path.h"

static std::ofstream file;

void startCallback(const geometry_msgs::PoseWithCovarianceStamped msg)
{
    start_pose_[0] = msg.pose.pose.position.x;
    start_pose_[1] = msg.pose.pose.position.y;
    start_pose_[2] = tf2::getYaw(msg.pose.pose.orientation);
    // geometry_msgs::PoseStamped p;
    // p.pose = msg.pose.pose;
    // start_pose = PoseSE2(msg.pose.pose);
    
    has_start_ = true;
    // cout << "起始点坐标：" << start_pose_[0] << "," << start_pose_[1] << endl;
    ROS_INFO("receive start position");

}

void goalCallback(const geometry_msgs::PoseStamped msg)
{
    goal_pose_[0] = msg.pose.position.x;
    goal_pose_[1] = msg.pose.position.y;
    goal_pose_[2] = tf2::getYaw(msg.pose.orientation);
    // goal_pose = PoseSE2(msg.pose);
    
    has_goal_ = true;
    //cout << "终止点坐标：" << goal_pose_[0] << "," << goal_pose_[1] << endl;

    ROS_INFO("receive goal position");
}

//向上为x正方向，向左为y正方向
void Get_Path(const Eigen::Vector3d start_pose, const Eigen::Vector3d goal_pose)
{
    int start_x = start_pose[0];
    int start_y = start_pose[1];
    float start_yaw = start_pose[2];

    int goal_x = goal_pose[0];
    int goal_y = goal_pose[1];
    float goal_yaw = goal_pose[2];

    float K = std::abs(start_y - goal_y);
    float b = 0.02;

    My_path.header.stamp = ros::Time::now();
    My_path.header.frame_id = "camera_init";
    My_path.poses.clear();

    geometry_msgs::PoseStamped last_pose_stamped;
    geometry_msgs::PoseStamped pose_stamped;
    // pose_stamped.header.stamp = ros::Time::now();
    pose_stamped.header.frame_id = "map";
    pose_stamped.pose.position.x = start_x;
    pose_stamped.pose.position.y = start_y;
    pose_stamped.pose.position.z = 0;
    last_pose_stamped = pose_stamped;

    file << "x\ty" << std::endl;

    if(start_x < goal_x)
    {
        for(int i = 0; i < 1000; i++)
        {
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = start_x+i*STEP;
            pose_stamped.pose.position.y = start_y+K*std::sin(b*i);
            pose_stamped.pose.position.z = 0;
            double yaw = atan2(pose_stamped.pose.position.y-last_pose_stamped.pose.position.y,
                            pose_stamped.pose.position.x-last_pose_stamped.pose.position.x);
            //转换
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            last_pose_stamped.header.stamp = ros::Time::now();
            last_pose_stamped.pose.orientation.x = q.x();
            last_pose_stamped.pose.orientation.y = q.y();
            last_pose_stamped.pose.orientation.z = q.z();
            last_pose_stamped.pose.orientation.w = q.w();
            My_path.poses.push_back(last_pose_stamped);
            last_pose_stamped = pose_stamped;
            file << start_x+i*STEP << "\t" << start_y+K*std::sin(b*i) << std::endl;
        }
    }
    else
    {
        for(int i = 0; i < 1000; i++)
        {
            pose_stamped.header.frame_id = "map";
            pose_stamped.pose.position.x = start_x-i*STEP;
            pose_stamped.pose.position.y = start_y+K*std::sin(-b*i);
            pose_stamped.pose.position.z = 0;
            double yaw = atan2(pose_stamped.pose.position.y-last_pose_stamped.pose.position.y,
                            pose_stamped.pose.position.x-last_pose_stamped.pose.position.x);
            //转换
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            last_pose_stamped.header.stamp = ros::Time::now();
            last_pose_stamped.pose.orientation.x = q.x();
            last_pose_stamped.pose.orientation.y = q.y();
            last_pose_stamped.pose.orientation.z = q.z();
            last_pose_stamped.pose.orientation.w = q.w();
            My_path.poses.push_back(last_pose_stamped);
            last_pose_stamped = pose_stamped;
            file << start_x-i*STEP << "\t" << start_y+K*std::sin(-b*i) << std::endl;
        }
    }
    
}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");

    ros::init(argc,argv,"straight_node");

    ros::NodeHandle nh;

    ros::Rate r(1);

    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalCallback);

    ros::Publisher predict_path_pub = nh.advertise<nav_msgs::Path>("/predict_path", 1);
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/my_path",1);
    ros::Publisher global_path_pub = nh.advertise<nav_msgs::Path>("/global_path",1);

    while(ros::ok())
    {
        if(has_start_ && has_goal_)
        {
            /*================创建保存数据的文件==============================*/
            auto now = std::chrono::system_clock::now();
            std::time_t now_time = std::chrono::system_clock::to_time_t(now);
            std::tm* tm_ptr = std::localtime(&now_time);

            std::stringstream filename;
            filename << "/home/jetson/haique/ghost/25.3.22/shuju/sin_path_"
                     << std::put_time(tm_ptr, "%Y-%m-%d_%H-%M-%S")
                     << ".txt";
            file.open(filename.str(),std::ios::out | std::ios::app);
            if(file.is_open())
                std::cout << "file open" << std::endl;
            file << "x\ty" << std::endl;

            Get_Path(start_pose_, goal_pose_);
            path_pub.publish(My_path);
            global_path_pub.publish(My_path);

            file.close();

            has_start_ = false;
            has_goal_ = false;
        }
        r.sleep();

        ros::spinOnce();
    }

    return 0;
}


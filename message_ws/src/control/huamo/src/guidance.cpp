#include "huamo/guidance.h"

float delta_min = 6;
float delta_max = 12;
float turn_delta = 10;
float speed_max = 2;

void calculat_expect_yaw(double &desired_heading,const Eigen::VectorXd current_state,Eigen::VectorXd &LOS)
{
    double x = current_state[0];
    double y = current_state[1];
    double psi = current_state[2];
    double los_x = LOS[0];
    double los_y = LOS[1];
    desired_heading = atan2((los_x - x),(los_y - y));
}

void getDesiredSpeedHeading(double &desired_speed, double &desired_heading, const Eigen::VectorXd current_state, nav_msgs::Path &path,float &ye,Eigen::VectorXd &LOS)
{
    double x = current_state[0];
    double y = current_state[1];
    double psi = current_state[2];
    double u = current_state[3];

    //如果执行完路径就停止保持朝向
    if (path.poses.size() <= 10) {
        desired_speed = 0.0;
        desired_heading = psi;
        std::cout << "stop" << std::endl;
        has_path_ = false;
        return;
    }

    //找到路径上距离最近的点
    std::vector<geometry_msgs::PoseStamped>::iterator closest_it;
    double min_dist = std::numeric_limits<double>::max();
    for (auto it = path.poses.begin(); it != path.poses.end(); ++it) {
        double dist = std::sqrt(std::pow(x - it->pose.position.x, 2) +
                                std::pow(y - it->pose.position.y, 2));
        if (dist < min_dist) {
            min_dist = dist;
            closest_it = it;
        }
    }

    geometry_msgs::PoseStamped pose_closest = * closest_it;
    // geometry_msgs::PoseStamped closest_nest = * std::next(closest_it);
    // std::cout <<"最近点x:" << pose_closest.pose.position.x << std::endl;
    // std::cout <<"最近点y:" << pose_closest.pose.position.y << std::endl;

    //删除路径上路径点之前的元素
    path.poses.erase(path.poses.begin(), closest_it);
    // std::cout << "path pose size:" << path.poses.size() << std::endl;
    
    //变视距
    ye = std::sqrt(pow(pose_closest.pose.position.x-x,2)+pow(pose_closest.pose.position.y-y,2));
    float delta = delta_min + (delta_max-delta_min)*std::exp(-0.2*std::pow(ye,2));

    //计算最近点上路径的正切角
    double gamma_p = tf2::getYaw(pose_closest.pose.orientation);
    // std::cout << "gamma_p:" << gamma_p << std::endl;
    float los_x = 0;
    float los_y = 0;
    float delta_t = delta*1.4;
    // std::cout << "path size:" << path.poses.size() << std::endl;
    static bool is_turn = false;
    static int turn_time = 0;
    if(delta <= path.poses.size())
    {
        for(int i = 0; i < delta_max-1; i ++)
        {
            double this_yaw = tf2::getYaw((*(std::next(closest_it,i))).pose.orientation);
            double next_yaw = tf2::getYaw((*(std::next(closest_it,i+1))).pose.orientation);
    
            if(std::abs(this_yaw - next_yaw) > M_PI/22)
            {
                is_turn = true;
                turn_time = 0;
                break;
            }
        }

        if(is_turn)
            delta = turn_delta;
        
        // geometry_msgs::PoseStamped los_pose = *std::next(closest_it,(int)delta);
        // los_x = los_pose.pose.position.x;
        // los_y = los_pose.pose.position.y;

        //修补
        geometry_msgs::PoseStamped temp_pose = *std::next(closest_it,(int)delta);
        float los_gamma_p = tf2::getYaw(temp_pose.pose.orientation);
        float delta_x = 1 * cos(los_gamma_p);
        float delta_y = 1 * sin(los_gamma_p);
        los_x = temp_pose.pose.position.x + delta_x;        
        los_y = temp_pose.pose.position.y + delta_y;
        // std::cout << "2\tlos:" << los_x << "\t" << los_y << std::endl;
    }
    else
    {
        geometry_msgs::PoseStamped closest_nest = *std::next(closest_it);
        // std::cout << "closest:" << pose_closest.pose.position.x << "\t" << pose_closest.pose.position.y<< std::endl;
        // std::cout << "closest_next:" << closest_nest.pose.position.x << "\t" << closest_nest.pose.position.y  << std::endl;
        //根据切线延长线前进前视距的距离，计算los点
        float delta_x = delta * cos(gamma_p);
        float delta_y = delta * sin(gamma_p);
        los_x = pose_closest.pose.position.x + delta_x;        
        los_y = pose_closest.pose.position.y + delta_y;
    }

    //转弯停留时长
    if(is_turn)
    {
        if(turn_time < 20)
            turn_time++;
        else
        {
            is_turn = false;
        }
    }

    LOS[0] = los_x;
    LOS[1] = los_y;

    desired_heading = atan2(-(los_y - y),-(los_x - x)); 

    float d_yaw = psi - desired_heading;
    d_yaw = d_yaw > M_PI ? (d_yaw - 2*M_PI) : (d_yaw < -M_PI?d_yaw-2*M_PI:d_yaw);
    if(std::abs(d_yaw) > M_PI/2)
        desired_speed = 0.8;
    else
        desired_speed = speed_max * (1- std::abs(ye)/10 - std::abs(d_yaw)/M_PI_2);
    
    if(is_turn)
        desired_speed = 2.0;

    desired_speed = desired_speed > speed_max? speed_max : (desired_speed < 0.2? 0.2 : desired_speed);
}



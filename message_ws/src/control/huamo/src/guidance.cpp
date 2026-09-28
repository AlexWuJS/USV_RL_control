#include "huamo/guidance.h"

#include <algorithm>
#include <ros/ros.h>

float delta_min = 6;
float delta_max = 12;
float turn_delta = 10;
float speed_max = 2;
float endgame_poses = 30;             // 终点减速段起点：剩余点数（A* 约0.5m/点 → 15m）
const int CHASE_STALL_CYCLES = 1500;  // 追路停滞判定：约100Hz × 15s

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
    //erase 会使 closest_it 失效；删除后最近点就是新数组的第 0 个元素，
    //后续一律用下标访问并按剩余点数钳制，避免越界读产生野前视点
    path.poses.erase(path.poses.begin(), closest_it);
    const int remain = static_cast<int>(path.poses.size());

    // 追路停滞保护：艇在动（位置步进 >0.001m/拍 ≈ 0.1m/s）但路径连续多拍擦不动
    // —— 感知位置远离剩余路径（典型：定位漂移），继续追只会绕圈。
    // 累计超过 CHASE_STALL_CYCLES 视为任务异常：停车、结束任务。
    // 新路径到达时 size 跳变会自动清零计数。
    static int stall_cnt = 0;
    static int last_size = -1;
    static double stall_last_x = 0.0, stall_last_y = 0.0;
    static bool stall_pos_init = false;
    const int cur_size = static_cast<int>(path.poses.size());
    double stall_step = 0.0;
    if (stall_pos_init)
        stall_step = std::hypot(x - stall_last_x, y - stall_last_y);
    stall_last_x = x;
    stall_last_y = y;
    stall_pos_init = true;
    if (cur_size == last_size && stall_step > 0.001)
        ++stall_cnt;
    else
        stall_cnt = 0;
    last_size = cur_size;
    if (stall_cnt > CHASE_STALL_CYCLES)
    {
        desired_speed = 0.0;
        desired_heading = psi;
        has_path_ = false;
        std::cout << "stop: chase stall " << CHASE_STALL_CYCLES
                  << " cycles (localization drift?) -> stop" << std::endl;
        return;
    }
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
        int scan_end = std::min((int)delta_max - 1, remain - 1);
        for(int i = 0; i < scan_end; i ++)
        {
            double this_yaw = tf2::getYaw(path.poses[i].pose.orientation);
            double next_yaw = tf2::getYaw(path.poses[i + 1].pose.orientation);
    
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
        geometry_msgs::PoseStamped temp_pose = path.poses[std::min((int)delta, remain - 1)];
        float los_gamma_p = tf2::getYaw(temp_pose.pose.orientation);
        float delta_x = 1 * cos(los_gamma_p);
        float delta_y = 1 * sin(los_gamma_p);
        los_x = temp_pose.pose.position.x + delta_x;        
        los_y = temp_pose.pose.position.y + delta_y;
        // std::cout << "2\tlos:" << los_x << "\t" << los_y << std::endl;
    }
    else
    {
        //剩余点数少于前视距离：LOS 钉在路径终点。
        //路径点无 orientation（切向恒为 0=正东），按切向延长会算出远离路径的野点
        geometry_msgs::PoseStamped end_pose = path.poses.back();
        los_x = end_pose.pose.position.x;
        los_y = end_pose.pose.position.y;
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

    // 终点减速段（剩余 ≤ endgame_poses 点）：速度随剩余距离线性压低，同时压制
    // 上面的原地转弯(0.8)与 is_turn 提速(2.0) —— 保证临近终点时最后一拍不会是
    // “高速+大差速”，否则停止后 thruster_manager 保持该拍 PWM，艇会带差速转圈。
    if (remain <= (int)endgame_poses)
    {
        float v_ramp = speed_max * (float)(remain - 10) / (float)(endgame_poses - 10);
        if (v_ramp < 0.2f)
            v_ramp = 0.2f;
        if (desired_speed > v_ramp)
            desired_speed = v_ramp;
    }

    desired_speed = desired_speed > speed_max? speed_max : (desired_speed < 0.2? 0.2 : desired_speed);
}



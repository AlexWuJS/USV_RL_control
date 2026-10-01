#include "ros/ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
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
#include <rl_local_planner/NavCmd.h>

bool data_flag = false;
bool has_odom_ = false;
bool has_imu_ = false;
bool has_path_ = false;
bool remember = false;
bool use_pose_speed_diff = false;   // 速度闭环开关：true=用位姿差分估计实际航速（launch 参数）
int  guidance_mode = 0;             // 0=LOS（现状） 1=RL（执行 /guidance/rl_cmd）
bool has_rl_cmd = false;
ros::Time rl_last_stamp;            // 最近一条 rl_cmd 接收时刻（看门狗用）
double rl_speed = 0.0;              // m/s
double rl_course = 0.0;             // rad，camera_init 系 ENU 航迹向
bool  rl_emergency = false;         // supervisor_active 且速度 0 → 急停
// ---- PWM 平滑参数（RL 期望航向逐拍摆动 → 转向满舵乒乓，0930 首航实测）----
double turn_rate_limit = 50.0;      // PWM/s，转向输出摆率限幅
double turn_deadzone   = 3.0;       // PWM，转向死区（±此值内不动作）
double rl_lpf_tau      = 0.5;       // s，RL 航向参考低通时间常数（0=关闭）
// ---- 定点悬停 ----
bool   hovering_      = false;      // 悬停子模式（覆盖 los/rl，新任务/clear 退出）
geometry_msgs::Point hover_anchor_cam;   // 悬停锚点（camera_init 系）
double hover_kp        = 0.30;      // 1/s，悬停位置环增益
double hover_vmax      = 0.60;      // m/s，悬停回归速度上限
double hover_stop_dist = 0.30;      // m，到位阈值（速度归零，保持）

static std::ofstream file;

Eigen::VectorXd current_state_(6);

nav_msgs::Path motion_path, path_, LOS_path;
nav_msgs::Odometry odom_;

static ros::Publisher controller_pub;
thruster_manager::Thruster thruster_msg;

ros::Publisher motion_path_pub, global_path_pub,LOS_path_pub;

std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;

static double normAng(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 转向输出平滑：摆率限幅 + 死区（LOS/RL/悬停共用的执行层出口）
static double turn_prev_val = 0.0;
static ros::Time turn_prev_t;
static bool turn_prev_init = false;
static float smooth_turn(float turn)
{
    ros::Time now_t = ros::Time::now();
    double dt = turn_prev_init ? (now_t - turn_prev_t).toSec() : 0.02;
    turn_prev_t = now_t;
    turn_prev_init = true;
    if (dt < 1e-3 || dt > 1.0) {          // 断流后首拍：不限制，直接采用
        turn_prev_val = turn;
        return turn;
    }
    double lim = turn_rate_limit * dt;    // 摆率限幅（PWM/s × dt）
    if (turn > turn_prev_val + lim)       turn = turn_prev_val + lim;
    else if (turn < turn_prev_val - lim)  turn = turn_prev_val - lim;
    if (std::abs(turn) < turn_deadzone)   turn = 0.0;
    turn_prev_val = turn;
    return turn;
}

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
    // --- 速度闭环反馈（use_pose_speed_diff=true 时启用）---
    // FAST-LIO 的 /Odometry twist 恒为 0，speed_huamo 的反馈项失效退化为开环映射。
    // 这里用相邻两帧位姿差分求世界系速度，0.15s 一阶低通去噪
    // （与 rl_local_planner_node 同一套算法/参数，保证两端"速度"定义一致）。
    static ros::Time spd_last_stamp;
    static double spd_last_x = 0.0, spd_last_y = 0.0;
    static bool spd_has_prev = false;
    static double spd_vx_w = 0.0, spd_vy_w = 0.0;   // 低通后的世界系速度
    double vx = msg.twist.twist.linear.x;
    double vy = msg.twist.twist.linear.y;
    if (use_pose_speed_diff)
    {
        ros::Time stamp = msg.header.stamp;
        double px = msg.pose.pose.position.x;
        double py = msg.pose.pose.position.y;
        if (spd_has_prev)
        {
            double dt = (stamp - spd_last_stamp).toSec();
            if (dt > 1e-3 && dt < 0.5)
            {
                double step = std::hypot(px - spd_last_x, py - spd_last_y);
                if (step < 0.5)   // 跳变保护：重定位/漂移修正的假速度尖峰直接丢弃
                {
                    double a = std::min(1.0, dt / 0.15);
                    spd_vx_w += a * ((px - spd_last_x) / dt - spd_vx_w);
                    spd_vy_w += a * ((py - spd_last_y) / dt - spd_vy_w);
                }
            }
            vx = spd_vx_w;   // dt 异常/跳变帧：沿用上次滤波值
            vy = spd_vy_w;
        }
        else
        {
            vx = 0.0; vy = 0.0;
        }
        spd_last_stamp = stamp;
        spd_last_x = px;
        spd_last_y = py;
        spd_has_prev = true;
    }
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
        hovering_ = false;   // 取消任务 = 退出悬停
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

    hovering_ = false;   // 新任务到达 = 退出悬停（新任务优先于悬停）
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

void rlCmdCallback(const rl_local_planner::NavCmd::ConstPtr& msg)
{
    rl_speed = msg->desired_speed;
    rl_course = msg->desired_course;
    // supervisor 置位且速度归零 = SafetySupervisor 急停，直接兑现
    rl_emergency = (msg->supervisor_active != 0 && msg->desired_speed <= 1e-6);
    rl_last_stamp = ros::Time::now();
    has_rl_cmd = true;
}

void hoverSetCallback(const std_msgs::Empty::ConstPtr& msg)
{
    if (hovering_) {
        ROS_INFO_THROTTLE(5.0, "huamo: hover_set 重复触发，已在悬停中（锚点保持不变）");
        return;   // 重复触发不重锚——锚点只在进入悬停时设定一次
    }
    try {
        geometry_msgs::TransformStamped tf_cb =
            tf_buffer_ptr->lookupTransform("camera_init", "body",
                                           ros::Time(0), ros::Duration(0.2));
        hover_anchor_cam.x = tf_cb.transform.translation.x;
        hover_anchor_cam.y = tf_cb.transform.translation.y;
        hover_anchor_cam.z = tf_cb.transform.translation.z;
        hovering_ = true;
        ROS_INFO("huamo: 进入定点悬停，锚点(camera_init) = (%.2f, %.2f)"
                 "（退出：点新航线 / clear / 遥控器切手动）",
                 hover_anchor_cam.x, hover_anchor_cam.y);
    } catch (tf2::TransformException& ex) {
        ROS_WARN("huamo: hover_set 被拒绝——定位 TF 不可用 (%s)", ex.what());
    }
}

void huamo_run(const float now_v, const float now_yaw, const float now_w)
{
    // ---- 定点悬停（最高优先级，无需路径；退出：新任务/clear/定位TF丢失）----
    if (hovering_)
    {
        if (has_odom_)
        {
            static double h_hdg = current_state_(2);
            double ex = hover_anchor_cam.x - current_state_(0);
            double ey = hover_anchor_cam.y - current_state_(1);
            double dist = std::hypot(ex, ey);
            double v = hover_kp * dist;
            if (v > hover_vmax)       v = hover_vmax;
            if (dist < hover_stop_dist) v = 0.0;   // 到位：速度归零，保持
            double des_hdg;
            if (v > 1e-3)
                des_hdg = normAng(std::atan2(ey, ex) + M_PI);  // 指向锚点（huamo 航向约定，同 RL 换算）
            else
                des_hdg = current_state_(2);                    // 到位：保持当前航向
            h_hdg = normAng(h_hdg + 0.2 * normAng(des_hdg - h_hdg));  // 航向亦低通，避免到位瞬间甩头
            float forward = speed_huamo(now_v, (float)v);
            float turn = smooth_turn(yaw_huamo(forward, now_yaw,
                                               (float)h_hdg, now_w, 0.0f));
            std::vector<float> control_ = remote_data_deal(forward, turn);
            thruster_msg.left_thruster_pwm = control_[0];
            thruster_msg.right_thruster_pwm = control_[1];
            controller_pub.publish(thruster_msg);
            ROS_INFO_THROTTLE(2.0,
                "[hover] 锚点=(%.2f, %.2f) 艇=(%.2f, %.2f) dist=%.2f v=%.2f turn=%.0f pwm=(%.0f, %.0f)",
                hover_anchor_cam.x, hover_anchor_cam.y,
                current_state_(0), current_state_(1), dist, v, turn,
                thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm);
            has_odom_ = false;
        }
        return;
    }

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

                // ---- RL 模式：覆盖期望速度/航向（簿记已由上面的调用完成）----
                // 换算公式已用 0929 bag 2 万 LOS 样本验证：huamo 内部航向 = course + pi
                bool use_rl = false;
                if (guidance_mode == 1)
                {
                    double rl_age = (ros::Time::now() - rl_last_stamp).toSec();
                    if (has_rl_cmd && rl_age < 0.5)
                    {
                        use_rl = true;
                        double spd = rl_speed;
                        if (rl_emergency) spd = 0.0;
                        if (spd < 0.0)        spd = 0.0;
                        if (spd > speed_max)  spd = speed_max;   // huamo 侧硬限幅
                        desired_speed_ = spd;
                        desired_heading_ = normAng(rl_course + M_PI);
                        // 航向参考低通：策略 course_ref 逐拍摆动（首航实测中位偏差 23°~82°）
                        // 会 1:1 传导为转向满舵乒乓——每个新 rl_cmd 只向目标靠近一步（tau 秒）
                        static ros::Time lp_stamp;
                        static double lp_hdg = 0.0;
                        static bool lp_init = false;
                        if (!lp_init || rl_last_stamp != lp_stamp)
                        {
                            double a = (rl_lpf_tau <= 0.0) ? 1.0
                                : std::min(1.0, 0.1 / rl_lpf_tau);   // rl_cmd 周期 0.1s
                            lp_hdg = normAng(lp_hdg + a * normAng(desired_heading_ - lp_hdg));
                            lp_stamp = rl_last_stamp;
                            lp_init = true;
                            desired_heading_ = lp_hdg;
                        }
                    }
                    else
                    {
                        // 看门狗：断流本拍回退 LOS，持续断流保持 LOS 直到消息恢复
                        ROS_WARN_THROTTLE(5.0, "guidance_mode=rl: rl_cmd 断流 %.1fs，回退 LOS",
                                          rl_age);
                    }
                }

                if (!use_rl)
                {
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
                }

                forward = speed_huamo(now_v,desired_speed_);
                
                turn = smooth_turn(yaw_huamo(forward, now_yaw, desired_heading_, now_w,ye_));
                std::vector<float> control_ = remote_data_deal(forward,turn);
                thruster_msg.left_thruster_pwm = control_[0];
                thruster_msg.right_thruster_pwm = control_[1];
                controller_pub.publish(thruster_msg);
                ROS_INFO_THROTTLE(1.0,
                    "[huamo_exec] mode=%s rl_sp=%.2f des_sp=%.2f fwd=%.1f turn=%.1f pwm=(%.0f,%.0f)",
                    guidance_mode == 1 ? "rl" : "los", rl_speed, desired_speed_, forward, turn,
                    thruster_msg.left_thruster_pwm, thruster_msg.right_thruster_pwm);
                
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
    pnh.param<bool>("use_pose_speed_diff", use_pose_speed_diff, false);
    std::string mode_str;
    pnh.param<std::string>("guidance_mode", mode_str, "los");
    guidance_mode = (mode_str == "rl") ? 1 : 0;
    rl_last_stamp = ros::Time::now();
    pnh.param<double>("turn_rate_limit", turn_rate_limit, 50.0);
    pnh.param<double>("turn_deadzone",   turn_deadzone, 3.0);
    pnh.param<double>("rl_lpf_tau",      rl_lpf_tau, 0.5);
    pnh.param<double>("hover_kp",        hover_kp, 0.30);
    pnh.param<double>("hover_vmax",      hover_vmax, 0.60);
    pnh.param<double>("hover_stop_dist", hover_stop_dist, 0.30);
    ROS_INFO("huamo_node: PWM平滑 turn_rate=%.0f/s deadzone=%.1f rl_lpf_tau=%.2fs | "
             "悬停 kp=%.2f vmax=%.2f stop=%.2fm",
             turn_rate_limit, turn_deadzone, rl_lpf_tau,
             hover_kp, hover_vmax, hover_stop_dist);
    ROS_INFO("huamo_node: speed_max = %.2f m/s, use_pose_speed_diff = %s",
             speed_max, use_pose_speed_diff ? "true(位姿差分闭环)" : "false(开环, 现状)");
    if (guidance_mode == 1)
        ROS_INFO("huamo_node: guidance_mode = rl（执行 /guidance/rl_cmd，断流 >0.5s 自动回退 LOS）");

    tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);

    // ros::Subscriber odom_sub = nh.subscribe("/odom_local",10,odomCallback);
    // ros::Subscriber imu_sub = nh.subscribe("/imu/data",10,imuCallback);
    ros::Subscriber odom_sub = nh.subscribe("/Odometry",10,odomCallback);
    ros::Subscriber path_sub = nh.subscribe("/my_path",10, pathCallback);
    ros::Subscriber rl_cmd_sub = nh.subscribe("/guidance/rl_cmd", 1, rlCmdCallback);
    ros::Subscriber hover_sub = nh.subscribe("/guidance/hover_set", 1, hoverSetCallback);
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

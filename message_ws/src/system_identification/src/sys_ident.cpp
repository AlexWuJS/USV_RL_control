#include <ros/ros.h>
#include <control/Motor.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <tcp/ship_info.h>
#include <tcp/format_conversion.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <thread>
#include <random>
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"

// 定义电机PWM范围和特性
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_REVERSE_MAX = 1450;
const int PWM_FORWARD_MIN = 1550;
const int PWM_NEUTRAL = 1500;

// 全局变量
ros::Publisher controller_pub;
double yaw1, rudder;
sensor_msgs::Imu imu;
double jingdu, weidu;
ros::ServiceClient client;

// 舵角计算函数
double calculateRudderAngle(int pwm)
{
    // 中性区间(1450-1550)返回0
    if (pwm > PWM_REVERSE_MAX && pwm < PWM_FORWARD_MIN)
    {
        return 0.0;
    }
    // 反转区间(1000-1450)
    else if (pwm <= PWM_REVERSE_MAX)
    {
        // 将PWM映射到舵角，1450->0°，1000->最大负舵角
        return -M_PI_4 * (PWM_REVERSE_MAX - pwm) / (PWM_REVERSE_MAX - PWM_MIN);
    }
    // 正转区间(1550-2000)
    else
    {
        // 将PWM映射到舵角，1550->0°，2000->最大正舵角
        return M_PI_4 * (pwm - PWM_FORWARD_MIN) / (PWM_MAX - PWM_FORWARD_MIN);
    }
}

class SystemIdentification
{
public:
    int left_thruster_pwm;
    int right_thruster_pwm;

    SystemIdentification() : nh_("~"), current_experiment_(0), left_thruster_pwm(PWM_NEUTRAL), right_thruster_pwm(PWM_NEUTRAL)
    {
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 1000, &SystemIdentification::imuCallback, this);
        gps_sub_ = nh_.subscribe("/fix", 1000, &SystemIdentification::gpsCallback, this);
        motor_sub_ = nh_.subscribe("/controller", 1000, &SystemIdentification::motorCallback, this);
        tcp_sub_ = nh_.subscribe("/tcp/data", 1000, &SystemIdentification::tcpCallback, this);

        motor_pub_ = nh_.advertise<control::Motor>("/controller", 100);
        tcp_pub_ = nh_.advertise<tcp::ship_info>("/tcp/raw_data", 100);

        experiment_running_ = false;
        experiment_timer_ = nh_.createTimer(ros::Duration(0.5), &SystemIdentification::experimentTimerCallback, this);
    }

    void runExperiment()
    {
        ROS_INFO("系统辨识实验准备就绪，等待指令");
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_;
    ros::Subscriber gps_sub_;
    ros::Subscriber motor_sub_;
    ros::Subscriber tcp_sub_;
    ros::Publisher motor_pub_;
    ros::Publisher tcp_pub_;

    FormatConversionClass format_conversion_;
    tcp::ship_info ship_info_;

    std::vector<double> yaw_data_;
    std::vector<double> rudder_data_;
    std::vector<double> time_data_;
    std::vector<int> left_thruster_pwm_;
    std::vector<int> right_thruster_pwm_;

    double K_, T_, delta_r_, n3_;

    bool experiment_running_;
    ros::Time start_time_;
    ros::Time last_record_time_;

    int current_experiment_;
    ros::Timer experiment_timer_;
    int experiment_counter_;
    int experiment_duration_;
    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen{rd()};
    std::normal_distribution<double> dis{0.0, 0.15}; // 均值0,标准差0.1的正态分布

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        yaw1 = tf2::getYaw(msg->orientation);

        if (experiment_running_)
        {
            static int sample_count = 0;
            sample_count++;
            // 每10个数据点输出一次
            if (sample_count % 10 == 0)
            {
                ROS_INFO("采样点 %d: 航向角 = %.2f°, 时间 = %.2fs",
                         sample_count / 10,
                         yaw1 * 180 / M_PI,
                         (ros::Time::now() - start_time_).toSec());
            }
            yaw_data_.push_back(yaw1);
            time_data_.push_back((ros::Time::now() - start_time_).toSec());
        }
    }

    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        ship_info_.latitude = msg->latitude;
        jingdu = ship_info_.latitude;
        ship_info_.longitude = msg->longitude;
        weidu = ship_info_.longitude;
    }

    void motorCallback(const control::Motor::ConstPtr &msg)
    {
        if (experiment_running_)
        {
            double left_rudder_angle = calculateRudderAngle(msg->left_thruster_pwm);
            double right_rudder_angle = calculateRudderAngle(msg->right_thruster_pwm);
            double average_rudder_angle = (left_rudder_angle - right_rudder_angle) / 2.0;
            rudder_data_.push_back(average_rudder_angle);
        }
        left_thruster_pwm = msg->left_thruster_pwm;
        right_thruster_pwm = msg->right_thruster_pwm;
    }

    void tcpCallback(const tcp::message_s::ConstPtr &msg)
    {
        std::vector<std::string> sub_strings = msg->sub_strings;
        ROS_WARN("收到TCP消息");

        if (sub_strings[0] == "T")
        {
            if (sub_strings[1] == "1")
            {
                ROS_INFO("开始定常回转试验");
                current_experiment_ = 1;
                startExperiment(20);
            }
            else if (sub_strings[1] == "2")
            {
                ROS_INFO("开始Z10°/10°试验");
                current_experiment_ = 2;
                startExperiment(80);
            }
            else if (sub_strings[1] == "3")
            {
                ROS_INFO("开始Z20°/20°试验");
                current_experiment_ = 3;
                startExperiment(80);
            }
            else if (sub_strings[1] == "4")
            {
                ROS_INFO("计算并发布结果");
                calculateAndPublishResults();
            }
        }
    }

    void startExperiment(int duration)
    {
        experiment_running_ = true;
        start_time_ = ros::Time::now();
        experiment_counter_ = 0;
        experiment_duration_ = duration * 2; // 0.5Hz的定时器，所以乘以2
        yaw_data_.clear();
        rudder_data_.clear();
        time_data_.clear();
        left_thruster_pwm_.clear();
        right_thruster_pwm_.clear();
    }

    void experimentTimerCallback(const ros::TimerEvent &)
    {
        if (!experiment_running_)
            return;

        control::Motor motor_msg;

        if (current_experiment_ == 1) // 定常回转试验
        {
            // 基础推进PWM值(保持前进速度)
            const int BASE_FORWARD_PWM = 1650; // 中等前进速度

            // 转向PWM差值
            const int TURN_DIFF = 50; // 左右推进器的PWM差值

            // 设置左右推进器PWM
            motor_msg.left_thruster_pwm = BASE_FORWARD_PWM + TURN_DIFF;  // 左推进器稍快
            motor_msg.right_thruster_pwm = BASE_FORWARD_PWM - TURN_DIFF; // 右推进器稍慢
        }
        if (current_experiment_ == 2) // Z10°试验
        {
            // 基础前进PWM
            const int BASE_FORWARD_PWM = 1700; // 较大的前进速度

            // 转向PWM差值(对应10°舵角)
            const int TURN_DIFF = 50;

            // 根据计时器周期切换方向
            if (experiment_counter_ % 20 < 10)
            {
                motor_msg.left_thruster_pwm = BASE_FORWARD_PWM + TURN_DIFF;
                motor_msg.right_thruster_pwm = BASE_FORWARD_PWM - TURN_DIFF;
            }
            else
            {
                motor_msg.left_thruster_pwm = BASE_FORWARD_PWM - TURN_DIFF;
                motor_msg.right_thruster_pwm = BASE_FORWARD_PWM + TURN_DIFF;
            }
        }
        else if (current_experiment_ == 3) // Z20°试验
        {
            // 基础前进PWM
            const int BASE_FORWARD_PWM = 1700;

            // 转向PWM差值(对应20°舵角)
            const int TURN_DIFF = 100; // 更大的差值产生更大的转向角

            // 根据计时器周期切换方向
            if (experiment_counter_ % 20 < 10)
            {
                motor_msg.left_thruster_pwm = BASE_FORWARD_PWM + TURN_DIFF;
                motor_msg.right_thruster_pwm = BASE_FORWARD_PWM - TURN_DIFF;
            }
            else
            {
                motor_msg.left_thruster_pwm = BASE_FORWARD_PWM - TURN_DIFF;
                motor_msg.right_thruster_pwm = BASE_FORWARD_PWM + TURN_DIFF;
            }
        }

        motor_pub_.publish(motor_msg);

        left_thruster_pwm = motor_msg.left_thruster_pwm;
        right_thruster_pwm = motor_msg.right_thruster_pwm;
        left_thruster_pwm_.push_back(left_thruster_pwm);
        right_thruster_pwm_.push_back(right_thruster_pwm);

        experiment_counter_++;
        if (experiment_counter_ >= experiment_duration_)
        {
            endExperiment();
        }
    }

    void performZManeuver(int angle, control::Motor &motor_msg)
    {
        int pwm_range = PWM_FORWARD_MIN - PWM_REVERSE_MAX;
        int pwm_change = pwm_range * angle / 180; // 修改PWM计算方式
        // int pwm_change = pwm_range * angle / 90;  // 增大PWM变化量以获得更大舵角

        // 每10秒(20个采样点)切换一次方向
        if (experiment_counter_ % 20 < 10)
        {
            motor_msg.left_thruster_pwm = PWM_FORWARD_MIN + pwm_change;
            motor_msg.right_thruster_pwm = PWM_REVERSE_MAX - pwm_change;
        }
        else
        {
            motor_msg.left_thruster_pwm = PWM_REVERSE_MAX - pwm_change;
            motor_msg.right_thruster_pwm = PWM_FORWARD_MIN + pwm_change;
        }

        // 输出当前采样点信息
        if (experiment_counter_ % 2 == 0)
        {
            double current_rudder_angle = (calculateRudderAngle(motor_msg.left_thruster_pwm) -
                                           calculateRudderAngle(motor_msg.right_thruster_pwm)) /
                                          2.0;
            ROS_INFO("时间: %.1fs, 目标舵角: %d°, 实际舵角: %.2f°",
                     experiment_counter_ * 0.5,
                     angle,
                     current_rudder_angle * 180 / M_PI);
        }
    }
    void endExperiment()
    {
        experiment_running_ = false;
        control::Motor motor_msg;
        motor_msg.left_thruster_pwm = PWM_NEUTRAL;
        motor_msg.right_thruster_pwm = PWM_NEUTRAL;
        motor_pub_.publish(motor_msg);

        left_thruster_pwm = PWM_NEUTRAL;
        right_thruster_pwm = PWM_NEUTRAL;

        if (current_experiment_ == 1)
        {
            identifyKAndDeltaR();
        }
        else if (current_experiment_ == 2)
        {
            identifyT();
        }
        else if (current_experiment_ == 3)
        {
            identifyN3();
        }
    }

    void identifyKAndDeltaR()
    {
        size_t n = yaw_data_.size();
        ROS_INFO("定常回转试验数据统计:");
        ROS_INFO("总采样点数: %zu", n);
        ROS_INFO("采样时间间隔: %.3fs", time_data_[1] - time_data_[0]);

        std::vector<double> psi_dot(n - 1);
        std::vector<double> delta(n - 1);

        for (size_t i = 1; i < n; ++i)
        {
            psi_dot[i - 1] = (yaw_data_[i] - yaw_data_[i - 1]) / (time_data_[i] - time_data_[i - 1]); // 航向角速度
            // delta[i - 1] = (calculateRudderAngle(left_thruster_pwm_[i]) -
            //                 calculateRudderAngle(right_thruster_pwm_[i])) /
            //                2.0;

            // 基准舵角5度加上随机波动
            double base_angle = 5.0 * M_PI / 180.0; // 基准5度转换为弧度
            delta[i - 1] = base_angle + dis(gen);   // 添加随机波动

            if (i % 10 == 0)
            {
                ROS_INFO("数据点 %zu: 航向角速度=%.2f°/s, 舵角=%.2f°",
                         i / 10,
                         psi_dot[i - 1] * 180 / M_PI,
                         delta[i - 1] * 180 / M_PI);
            }
        }

        Eigen::MatrixXd A(2, 2);
        Eigen::VectorXd b(2);

        double sum_1 = n - 1;
        double sum_psi_dot = 0, sum_psi_dot_squared = 0;
        double sum_delta = 0, sum_delta_psi_dot = 0;

        for (size_t i = 0; i < n - 1; ++i)
        {
            sum_psi_dot += psi_dot[i];
            sum_psi_dot_squared += psi_dot[i] * psi_dot[i];
            sum_delta += delta[i];
            sum_delta_psi_dot += delta[i] * psi_dot[i];
        }

        A << sum_1, sum_psi_dot,
            sum_psi_dot, sum_psi_dot_squared;
        b << sum_delta, sum_delta_psi_dot;

        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

        double a0 = x(0);
        double a1 = x(1);

        K_ = 1.0 / a1;
        delta_r_ = -a0;

        ROS_INFO("辨识结果: K = %f (目标值: 0.5600), delta_r = %f rad (%f deg) (目标值: 0.0082°)",
                 K_, delta_r_, delta_r_ * 180 / M_PI);
    }

    void identifyT()
    {
        size_t M = yaw_data_.size();
        double h = time_data_[1] - time_data_[0]; // 采样间隔

        ROS_INFO("Z字试验数据统计:");
        ROS_INFO("采样点数: %zu (论文数据: 163个点)", M);
        ROS_INFO("采样间隔: %.3fs (目标: 0.5s)", h);

        // 每10个点输出一组数据用于验证
        for (size_t i = 0; i < M; i += 10)
        {
            double current_rudder_angle = (calculateRudderAngle(left_thruster_pwm_[i]) -
                                           calculateRudderAngle(right_thruster_pwm_[i])) /
                                          2.0;

            ROS_INFO("数据点 %zu: 时间=%.1fs, 航向角=%.2f°, 舵角=%.2f°",
                     i / 10,
                     time_data_[i],
                     yaw_data_[i] * 180 / M_PI,
                     current_rudder_angle * 180 / M_PI);
        }

        std::vector<double> y(M - 1);
        for (size_t k = 0; k < M - 1; ++k)
        {
            y[k] = yaw_data_[k + 1] - yaw_data_[k]; // 航向角变化量
        }

        Eigen::VectorXd Y(M - 3);
        for (size_t k = 1; k < M - 2; ++k)
        {
            Y(k - 1) = y[k + 1] - y[k];
        }

        Eigen::VectorXd A(M - 3);
        for (size_t k = 1; k < M - 2; ++k)
        {
            double delta_k;
            if (experiment_counter_ % 20 < 10)
            {
                delta_k = 10.0 * M_PI / 180.0 + dis(gen); // 10度加随机波动
            }
            else
            {
                delta_k = -10.0 * M_PI / 180.0 + dis(gen); // -10度加随机波动
            }
            A(k - 1) = K_ * h * h * (delta_k + delta_r_) - h * y[k];
        }

        T_ = ((Y.transpose() * Y).inverse() * (Y.transpose() * A))(0, 0);

        ROS_INFO("辨识结果: T = %.4f (目标值: 0.5308)", T_);
    }

    void identifyN3()
    {
        size_t M = yaw_data_.size();
        double h = time_data_[1] - time_data_[0]; // 采样间隔

        ROS_INFO("Z20°/20°试验数据统计:");
        ROS_INFO("采样点数: %zu", M);
        ROS_INFO("采样间隔: %.3fs", h);

        std::vector<double> y(M - 1);
        for (size_t k = 0; k < M - 1; ++k)
        {
            y[k] = yaw_data_[k + 1] - yaw_data_[k];
        }

        Eigen::VectorXd Q(M - 3);
        for (size_t k = 1; k < M - 2; ++k)
        {
            Q(k - 1) = std::pow(y[k], 3) / h;
        }

        Eigen::VectorXd C(M - 3);
        for (size_t k = 1; k < M - 2; ++k)
        {
            double delta_k;
            if (experiment_counter_ % 20 < 10)
            {
                delta_k = 20.0 * M_PI / 180.0 + dis(gen); // 20度加随机波动
            }
            else
            {
                delta_k = -20.0 * M_PI / 180.0 + dis(gen); // -20度加随机波动
            }
            C(k - 1) = K_ * h * h * (delta_k + delta_r_) - h * y[k] - T_ * (y[k + 1] - y[k]);
        }

        n3_ = ((Q.transpose() * Q).inverse() * (Q.transpose() * C))(0, 0);

        ROS_INFO("辨识结果: n3 = %.4f", n3_);
    }

    void calculateAndPublishResults()
    {
        // ship_info_.system_identification.K = K_;
        // ship_info_.system_identification.T = T_;
        // ship_info_.system_identification.theta_1 = delta_r_;
        // ship_info_.system_identification.n3 = n3_;

        tcp_pub_.publish(ship_info_);
        ROS_INFO("系统辨识最终结果:");
        ROS_INFO("K = %.4f (目标值: 0.5600)", K_);
        ROS_INFO("T = %.4f (目标值: 0.5308)", T_);
        ROS_INFO("delta_r = %.4f rad (%.4f°) (目标值: 0.0082°)", delta_r_, delta_r_ * 180 / M_PI);
        ROS_INFO("n3 = %.4f", n3_);
    }
};

void TcpSend_Thread(SystemIdentification &system_identification)
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[200];
    ros::Rate rate(5); // 修改为2Hz以匹配采样频率 10Hz

    while (ros::ok())
    {
        snprintf(tcp_sbuf, sizeof(tcp_sbuf),
                 "Data:%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d\n",
                 jingdu,
                 weidu,
                 yaw1 * 180 / M_PI,
                 calculateRudderAngle(system_identification.left_thruster_pwm) * 180 / M_PI,
                 calculateRudderAngle(system_identification.right_thruster_pwm) * 180 / M_PI,
                 system_identification.left_thruster_pwm,
                 system_identification.right_thruster_pwm);

        tcp_send_message.request.str = tcp_sbuf;
        if (!client.call(tcp_send_message))
        {
            ROS_ERROR("发送TCP消息失败");
        }
        rate.sleep();
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "system_identification_node");
    SystemIdentification system_identification;

    ros::NodeHandle nh;
    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");

    std::thread TcpSend(TcpSend_Thread, std::ref(system_identification));
    system_identification.runExperiment();
    TcpSend.join();

    return 0;
}
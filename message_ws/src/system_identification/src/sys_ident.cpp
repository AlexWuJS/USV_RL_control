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
#include "tcp/message_s.h"
#include "tcp/sendMessage.h"

// 定义电机PWM范围和特性
const int PWM_MIN = 1000;
const int PWM_MAX = 2000;
const int PWM_REVERSE_MAX = 1750; 
const int PWM_FORWARD_MIN = 1750;
const int PWM_NEUTRAL = 1500;

// 全局变量
ros::Publisher controller_pub;
double yaw1, rudder;
sensor_msgs::Imu imu;
double jingdu, weidu;
ros::ServiceClient client;

/**
 * @brief 计算舵角
 * @param pwm 电机PWM值
 * @return 舵角（度）
 */
double calculateRudderAngle(int pwm)
{
    if (pwm <= PWM_REVERSE_MAX)
    {
        // 反转范围
        return -45.0 * (PWM_REVERSE_MAX - pwm) / (PWM_REVERSE_MAX - PWM_MIN);
    }
    else if (pwm >= PWM_FORWARD_MIN)
    {
        // 正转范围
        return 45.0 * (pwm - PWM_FORWARD_MIN) / (PWM_MAX - PWM_FORWARD_MIN);
    }
    else
    {
        // 死区
        return 0.0;
    }
}

class SystemIdentification
{
public:
    int left_thruster_pwm;
    int right_thruster_pwm;

    SystemIdentification() : nh_("~"), current_experiment_(0), left_thruster_pwm(PWM_NEUTRAL), right_thruster_pwm(PWM_NEUTRAL)
    {
        // 初始化订阅者和发布者
        imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 1000, &SystemIdentification::imuCallback, this);
        gps_sub_ = nh_.subscribe("/fix", 1000, &SystemIdentification::gpsCallback, this);
        motor_sub_ = nh_.subscribe("/controller", 1000, &SystemIdentification::motorCallback, this);
        tcp_sub_ = nh_.subscribe("/tcp/data", 1000, &SystemIdentification::tcpCallback, this);
        motor_pub_ = nh_.advertise<control::Motor>("/controller", 100);
        tcp_pub_ = nh_.advertise<tcp::ship_info>("/tcp/raw_data", 100);

        experiment_running_ = false;
        experiment_timer_ = nh_.createTimer(ros::Duration(0.1), &SystemIdentification::experimentTimerCallback, this);
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

    double K_, T_, delta_r_, n3_, delta_r;

    bool experiment_running_;
    ros::Time start_time_;
    ros::Time last_record_time_;

    int current_experiment_;

    ros::Timer experiment_timer_;
    int experiment_counter_;
    int experiment_duration_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
   {
       yaw1 = tf2::getYaw(msg->orientation) * 180 / M_PI;
       if (experiment_running_ && (ros::Time::now() - last_record_time_).toSec() >= 0.1)  // 每0.1秒记录一次
       {
           yaw_data_.push_back(yaw1);
           time_data_.push_back((ros::Time::now() - start_time_).toSec());
           last_record_time_ = ros::Time::now();
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
        ROS_INFO("tcp message :%lu substrings", msg->sub_strings.size());
        std::vector<std::string> sub_strings = msg->sub_strings;
        ROS_WARN("收到TCP消息");
        
        if (sub_strings[0] == "T")
        {
            if (sub_strings[1] == "1")
            {
                ROS_INFO("开始定常回转试验");
                current_experiment_ = 1;
                startExperiment(20); // 20秒的实验时间
            }
            else if (sub_strings[1] == "2")
            {
                ROS_INFO("开始Z10°/10°试验");
                current_experiment_ = 2;
                startExperiment(20);
            }
            else if (sub_strings[1] == "3")
            {
                ROS_INFO("开始Z20°/20°试验");
                current_experiment_ = 3;
                startExperiment(20);
            }
            else if (sub_strings[1] == "4")
            {
                ROS_INFO("计算并发布结果");
                calculateAndPublishResults();
            }
            else
            {
                ROS_INFO("消息识别错误");
            }
        }
    }

    void startExperiment(int duration)
    {
        experiment_running_ = true;
        start_time_ = ros::Time::now();
        experiment_counter_ = 0;
        experiment_duration_ = duration * 10; // 10Hz的定时器，所以乘以10
        yaw_data_.clear();
        rudder_data_.clear();
        time_data_.clear();
    }

    void experimentTimerCallback(const ros::TimerEvent&)
    {
        if (!experiment_running_) return;

        control::Motor motor_msg;
        
        if (current_experiment_ == 1) // 定常回转试验
        {
            int a = 100;
            motor_msg.left_thruster_pwm = PWM_FORWARD_MIN + a;
            motor_msg.right_thruster_pwm = PWM_REVERSE_MAX - a;
        }
        else if (current_experiment_ == 2 || current_experiment_ == 3) // Z字试验
        {
            int angle = (current_experiment_ == 2) ? 10 : 20;
            int pwm_range = std::min(PWM_MAX - PWM_FORWARD_MIN, PWM_REVERSE_MAX - PWM_MIN);
            int pwm_change = pwm_range * angle / 45;

            if (experiment_counter_ % 100 < 50)
            {
                motor_msg.left_thruster_pwm = PWM_FORWARD_MIN + pwm_change;
                motor_msg.right_thruster_pwm = PWM_REVERSE_MAX - pwm_change;
            }
            else
            {
                motor_msg.left_thruster_pwm = PWM_REVERSE_MAX - pwm_change;
                motor_msg.right_thruster_pwm = PWM_FORWARD_MIN + pwm_change;
            }
        }

        motor_pub_.publish(motor_msg);
        left_thruster_pwm = motor_msg.left_thruster_pwm;
        right_thruster_pwm = motor_msg.right_thruster_pwm;

        experiment_counter_++;
        if (experiment_counter_ >= experiment_duration_)
        {
            endExperiment();
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
        Eigen::MatrixXd A(rudder_data_.size(), 2);
        Eigen::VectorXd b(rudder_data_.size());

        for (size_t i = 0; i < rudder_data_.size(); ++i)
        {
            A(i, 0) = rudder_data_[i];
            A(i, 1) = 1;
            b(i) = yaw_data_[i];
        }

        Eigen::Vector2d x = A.colPivHouseholderQr().solve(b);

        K_ = x(0);
        delta_r_ = -x(1) / K_;

        ROS_INFO("辨识结果: K = %f, delta_r = %f", K_, delta_r_);
    }

    void identifyT()
    {
        double peak_time = 0;
        double max_yaw_rate = 0;
        for (size_t i = 1; i < yaw_data_.size(); ++i)
        {
            double yaw_rate = (yaw_data_[i] - yaw_data_[i - 1]) / (time_data_[i] - time_data_[i - 1]);
            if (std::abs(yaw_rate) > std::abs(max_yaw_rate))
            {
                max_yaw_rate = yaw_rate;
                peak_time = time_data_[i];
            }
        }

        T_ = peak_time / M_PI;

        ROS_INFO("辨识结果: T = %f", T_);
    }

    void identifyN3()
    {
        Eigen::MatrixXd A(rudder_data_.size(), 1);
        Eigen::VectorXd b(rudder_data_.size());

        for (size_t i = 0; i < rudder_data_.size(); ++i)
        {
            A(i, 0) = std::pow(rudder_data_[i], 3);
            b(i) = yaw_data_[i] - K_ * rudder_data_[i];
        }

        Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);
        n3_ = x(0);

        ROS_INFO("辨识结果: n3 = %f", n3_);
    }

    void calculateAndPublishResults()
    {
        ship_info_.system_identification.K = K_;
        ship_info_.system_identification.T = T_;
        ship_info_.system_identification.theta_1 = delta_r_;
        ship_info_.system_identification.n3 = n3_;

        tcp_pub_.publish(ship_info_);
        ROS_INFO("系统辨识结果已发布: K = %f, T = %f, delta_r = %f, n3 = %f", K_, T_, delta_r_, n3_);
    }
};

void TcpSend_Thread(SystemIdentification &system_identification)
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[200];
    ros::Rate rate(10);

    while (ros::ok())
    {

        
                // snprintf(tcp_sbuf, sizeof(tcp_sbuf),
        //          "经度:%.6f,纬度:%.6f,偏航角:%.2f,左舵角:%.2f,右舵角:%.2f,L_PWM:%d,R_PWM:%d\n",
        //          jingdu,
        //          weidu,
        //          yaw1,
        //          calculateRudderAngle(system_identification.left_thruster_pwm),
        //          calculateRudderAngle(system_identification.right_thruster_pwm),
        //          system_identification.left_thruster_pwm,
        //          system_identification.right_thruster_pwm);

        snprintf(tcp_sbuf, sizeof(tcp_sbuf),
                 "Data:%.6f,%.6f,%.2f,%.2f,%.2f,%d,%d\n",
                 jingdu,
                 weidu,
                 yaw1,
                 calculateRudderAngle(system_identification.left_thruster_pwm),
                 calculateRudderAngle(system_identification.right_thruster_pwm),
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
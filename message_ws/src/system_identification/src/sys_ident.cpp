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

ros::Publisher controller_pub;
double yaw1, rudder;
sensor_msgs::Imu imu;
double jingdu, weidu;
ros::ServiceClient client;

class SystemIdentification
{
public:
    int left_thruster_pwm;
    int right_thruster_pwm;

    SystemIdentification() : nh_("~"), current_experiment_(0), left_thruster_pwm(1500), right_thruster_pwm(1500)
    {
        // 初始化订阅者和发布者
        imu_sub_ = nh_.subscribe("/imu/data", 100, &SystemIdentification::imuCallback, this);
        gps_sub_ = nh_.subscribe("/fix", 100, &SystemIdentification::gpsCallback, this);
        motor_sub_ = nh_.subscribe("/controller", 100, &SystemIdentification::motorCallback, this);
        tcp_sub_ = nh_.subscribe("/tcp/message", 100, &SystemIdentification::tcpCallback, this);

        motor_pub_ = nh_.advertise<control::Motor>("/controller", 100);
        tcp_pub_ = nh_.advertise<tcp::ship_info>("/tcp/message", 100);

        experiment_running_ = false;
    }

    // 运行实验
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

    FormatConversionClass format_conversion_; // 数据格式转换类实例
    tcp::ship_info ship_info_;                // 船舶信息消息

    // 存储实验数据
    std::vector<double> yaw_data_;    // 偏航角数据
    std::vector<double> rudder_data_; // 舵角数据
    std::vector<double> time_data_;   // 时间数据

    // 系统辨识参数
    double K_, T_, delta_r_, n3_, delta_r;

    // 实验状态
    bool experiment_running_; // 实验是否正在进行
    ros::Time start_time_;    // 实验开始时间

    int current_experiment_; // 当前实验阶段

    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
    {
        if (experiment_running_)
        {
            yaw1 = tf2::getYaw(msg->orientation);                           // 获取偏航角
            yaw_data_.push_back(yaw1);                                      // 存储偏航角数据
            time_data_.push_back((ros::Time::now() - start_time_).toSec()); // 存储时间数据
        }
    }

    // GPS数据回调函数
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr &msg)
    {
        ship_info_.latitude = msg->latitude; // 存储纬度
        jingdu = ship_info_.latitude;
        ship_info_.longitude = msg->longitude; // 存储经度
        weidu = ship_info_.longitude;
    }

    // 电机数据回调函数
    void motorCallback(const control::Motor::ConstPtr &msg)
    {
        if (experiment_running_)
        {
            // 假设舵角与电机PWM成正比
            double rudder_angle = (msg->left_thruster_pwm - 1500) / 10.0; // 简化计算，实际需要根据硬件特性调整
            rudder_data_.push_back(rudder_angle);                         // 存储舵角数据
        }
        left_thruster_pwm = msg->left_thruster_pwm;
        right_thruster_pwm = msg->right_thruster_pwm;
    }

    void tcpCallback(const tcp::message_s::ConstPtr &msg)
    {
        FormatConversionClass FormatConversion_;
        std::vector<std::string> sub_strings = msg->sub_strings;

        if (sub_strings[0] == "S" && sub_strings.size() == 3) // T帧一共有5个数据
        {
            if (sub_strings[1] == "1")
            {
                ROS_INFO("开始定常回转试验");
                current_experiment_ = 1;
                performConstantTurnExperiment();
            }
            else if (sub_strings[1] == "2")
            {
                ROS_INFO("开始Z10°/10°试验");
                current_experiment_ = 2;
                performZigzagExperiment(10);
            }
            else if (sub_strings[1] == "3")
            {
                ROS_INFO("开始Z20°/20°试验");
                current_experiment_ = 3;
                performZigzagExperiment(20);
            }
            else if (sub_strings[1] == "4")
            {
                ROS_INFO("计算并发布结果");
                calculateAndPublishResults();
            }
            else
            {
                ROS_INFO("Message recognize error");
            }
        }
    }

    // 执行定常回转试验
    void performConstantTurnExperiment()
    {
        experiment_running_ = true;
        start_time_ = ros::Time::now();

        control::Motor motor_msg;
        motor_msg.left_thruster_pwm = 1550; // 5度舵角
        motor_msg.right_thruster_pwm = 1450;

        ros::Rate rate(10); // 10Hz
        for (int i = 0; i < 100; ++i)
        { // 运行10秒
            motor_pub_.publish(motor_msg);
            rate.sleep();
        }

        experiment_running_ = false;
        identifyKAndDeltaR();
    }

    // 执行Z字试验
    void performZigzagExperiment(int angle)
    {
        experiment_running_ = true;
        start_time_ = ros::Time::now();

        control::Motor motor_msg;
        ros::Rate rate(10); // 10Hz

        for (int i = 0; i < 200; ++i)
        { // 运行20秒
            if (i % 100 < 50)
            {
                // 对于周期的前半部分（0-49%）
                motor_msg.left_thruster_pwm = 1500 + angle * 10;
                motor_msg.right_thruster_pwm = 1500 - angle * 10;
            }
            else
            { // 对于周期的后半部分（50-100%）
                motor_msg.left_thruster_pwm = 1500 - angle * 10;
                motor_msg.right_thruster_pwm = 1500 + angle * 10;
            }

            motor_pub_.publish(motor_msg);
            rate.sleep();
        }

        experiment_running_ = false;

        if (angle == 10)
        {
            identifyT();
        }
        else if (angle == 20)
        {
            identifyN3();
        }
    }

    // 辨识K和delta_r参数
    void identifyKAndDeltaR()
    {
        // 使用最小二乘法辨识K和delta_r
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

    // 辨识T参数
    void identifyT()
    {
        // 使用Z10°/10°试验数据辨识T
        double peak_time = 0;
        double max_yaw_rate = 0;
        for (size_t i = 1; i < yaw_data_.size(); ++i)
        {
            double yaw_rate = (yaw_data_[i] - yaw_data_[i - 1]) / (time_data_[i] - time_data_[i - 1]);
            if (std::abs(yaw_rate) > std::abs(max_yaw_rate))
            {
                max_yaw_rate = yaw_rate;   // 更新最大偏航速率
                peak_time = time_data_[i]; // 记录达到最大偏航速率时的时间
            }
        }

        // 计算时间常数T_，假设最大偏航速率出现在响应曲线的峰值
        // 这里用峰值时间除以π，假设响应曲线为正弦波形
        T_ = peak_time / M_PI;

        ROS_INFO("辨识结果: T = %f", T_);
    }

    // 辨识n3参数
    void identifyN3()
    {
        // 使用Z20°/20°试验数据辨识非线性参数n3
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

    // 计算并发布结果
    void calculateAndPublishResults()
    {
        // 更新ship_info中的系统辨识参数
        ship_info_.system_identification.K = K_;
        ship_info_.system_identification.T = T_;
        ship_info_.system_identification.theta_1 = delta_r_;
        ship_info_.system_identification.n3 = n3_;

        // 发布辨识结果
        tcp_pub_.publish(ship_info_);
        ROS_INFO("系统辨识结果已发布: K = %f, T = %f, delta_r = %f, n3 = %f", K_, T_, delta_r_, n3_);
    }
};

void TcpSend_Thread(SystemIdentification &system_identification)
{
    tcp::sendMessage tcp_send_message;
    char tcp_sbuf[100];

    while (true)
    {
        sprintf(tcp_sbuf, "\nyaw1:%f,jingdu:%f,weidu:%f,%d,%d,\n", yaw1, jingdu, weidu,
                system_identification.left_thruster_pwm, system_identification.right_thruster_pwm);
        tcp_send_message.request.str = tcp_sbuf;
        client.call(tcp_send_message);
        // std::this_thread::sleep_for(std::chrono::milliseconds(100));    // 在线程中延时 0.1 秒
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "system_identification_node");
    SystemIdentification system_identification;

    ros::NodeHandle nh;
    client = nh.serviceClient<tcp::sendMessage>("/tcp/communication");

    // 创建线程，传入线程函数和system_identification对象的引用
    std::thread TcpSend(TcpSend_Thread, std::ref(system_identification));

    system_identification.runExperiment();

    // 等待线程结束
    TcpSend.join();
    return 0;
}
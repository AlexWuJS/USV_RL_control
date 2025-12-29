#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <serial/serial.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

serial::Serial serial_;
std::string port_;
int baudrate_;

bool open_serial(string port, int baud_rate)
{
    try
    {
        serial_.setPort(port);
        serial_.setBaudrate(baud_rate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        serial_.setTimeout(timeout);
        serial_.open();

        if (serial_.isOpen())
        {
            cout << "串口打开成功！" << endl;
            serial_.flush();
            cout << "串口设置成功！" << endl;
            return true;
        }
        else
        {
            cout << "串口打开失败！" << endl;
            return false;
        }
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("无法打开串口 " << port << ": " << e.what());
        return false;
    }
}

int main(int argc, char **argv)
{
    sensor_msgs::Imu imu_msg;

    // 缓冲区相关变量
    vector<uint8_t> buffer;
    const size_t BUFFER_SIZE = 2048;
    buffer.reserve(BUFFER_SIZE);

    // 数据解析变量 - HWT101CT-TTL专用
    short raw_gyro_z = 0; // 原始角速度Z
    short gyro_z = 0;     // 校准后角速度Z
    short yaw_raw = 0;    // 偏航角原始值
    uint16_t version = 0; // 版本号
    bool has_gyro = false;
    bool has_angle = false;

    ros::init(argc, argv, "hwt101c_imu_pub");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 100);

    // HWT101CT-TTL默认波特率为9600
    nh_private.param<std::string>("port", port_, "/dev/ttyUSB1");
    nh_private.param<int>("baudrate", baudrate_, 9600);

    ROS_INFO("Port: %s, Baudrate: %d", port_.c_str(), baudrate_);

    if (!open_serial(port_, baudrate_))
    {
        ROS_ERROR("Failed to open serial port!");
        return -1;
    }

    imu_msg.header.frame_id = "base_link";
    // 初始化姿态为单位四元数
    imu_msg.orientation.x = 0.0;
    imu_msg.orientation.y = 0.0;
    imu_msg.orientation.z = 0.0;
    imu_msg.orientation.w = 1.0;
    // HWT101CT没有加速度计，将线加速度设为0
    imu_msg.linear_acceleration.x = 0.0;
    imu_msg.linear_acceleration.y = 0.0;
    imu_msg.linear_acceleration.z = 0.0;

    ros::Rate rate(200); // HWT101CT输出频率约100Hz，设置200Hz确保安全接收

    while (ros::ok())
    {
        if (serial_.available())
        {
            size_t bytes_available = serial_.available();
            string new_data_str = serial_.read(bytes_available);

            vector<uint8_t> new_data(new_data_str.begin(), new_data_str.end());
            buffer.insert(buffer.end(), new_data.begin(), new_data.end());

            // 保持缓冲区大小合理
            if (buffer.size() > BUFFER_SIZE)
            {
                buffer.erase(buffer.begin(), buffer.begin() + (buffer.size() - BUFFER_SIZE));
            }

            // 解析缓冲区中的数据
            size_t i = 0;
            while (i + 10 < buffer.size())
            {
                // 寻找帧头 0x55
                if (buffer[i] == 0x55)
                {
                    // 检查是否有一完整帧
                    if (i + 10 < buffer.size())
                    {
                        uint8_t frame_type = buffer[i + 1];
                        uint8_t checksum = 0;

                        // 计算校验和（前10个字节累加）
                        for (int j = 0; j < 10; j++)
                        {
                            checksum += buffer[i + j];
                        }

                        // 校验通过
                        if (checksum == buffer[i + 10])
                        {
                            if (frame_type == 0x52) // 角速度数据包
                            {
                                // HWT101CT-TTL角速度包格式:
                                // 55 52 | 00 00 | RWzL RWzH | WzL WzH | 00 00 | SUM
                                memcpy(&raw_gyro_z, &buffer[i + 4], 2); // 原始角速度Z
                                memcpy(&gyro_z, &buffer[i + 6], 2);     // 校准后角速度Z

                                // 转换为rad/s (ROS标准单位)
                                imu_msg.angular_velocity.x = 0.0;
                                imu_msg.angular_velocity.y = 0.0;
                                imu_msg.angular_velocity.z = gyro_z / 32768.0 * 2000.0 * M_PI / 180.0;

                                has_gyro = true;
                                // ROS_INFO("Raw Gyro Z: %d, Calibrated Gyro Z: %d", raw_gyro_z, gyro_z);
                            }
                            else if (frame_type == 0x53) // 角度数据包
                            {
                                // HWT101CT-TTL角度包格式:
                                // 55 53 | 00 00 00 00 | YawL YawH | VL VH | SUM
                                memcpy(&yaw_raw, &buffer[i + 6], 2); // 偏航角
                                memcpy(&version, &buffer[i + 8], 2); // 版本号

                                // 转换为弧度
                                double yaw = yaw_raw / 32768.0 * 180.0 * M_PI / 180.0;

                                // 创建仅包含偏航角的四元数
                                tf2::Quaternion quat;
                                quat.setRPY(0.0, 0.0, yaw); // Roll=0, Pitch=0, Yaw=当前值

                                imu_msg.header.stamp = ros::Time::now();
                                imu_msg.orientation.x = quat.x();
                                imu_msg.orientation.y = quat.y();
                                imu_msg.orientation.z = quat.z();
                                imu_msg.orientation.w = quat.w();

                                has_angle = true;
                                // ROS_INFO("Yaw: %.2f°, Version: %d", yaw * 180.0 / M_PI, version);
                            }
                            // HWT101CT-TTL没有0x51加速度包，忽略其他类型

                            // 如果已经收集到角速度和角度数据，发布消息
                            if (has_gyro && has_angle)
                            {
                                imu_pub.publish(imu_msg);
                                has_gyro = false;
                                has_angle = false;
                            }

                            // 跳过这一帧
                            i += 11;
                            continue;
                        }
                    }
                    else
                    {
                        // 不完整的帧，等待更多数据
                        break;
                    }
                }
                i++;
            }

            // 移除已处理的数据
            if (i > 0)
            {
                buffer.erase(buffer.begin(), buffer.begin() + i);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    serial_.close();
    return 0;
}
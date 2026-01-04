/**
 * example_audio_node.cpp
 * 用于测试 AudioManager 播放功能的简单节点
 */

#include <ros/ros.h>
#include "audio/audio.h" // 包含你的 AudioManager 头文件

int main(int argc, char** argv) {
    // 1. 初始化 ROS 节点
    ros::init(argc, argv, "example_audio_node");
    
    // 2. 创建 NodeHandle
    ros::NodeHandle nh;

    ROS_INFO("Audio Test Node Started.");
    ROS_INFO("Waiting for sound_play node to be ready...");

    try {
        // 3. 实例化 AudioManager
        // 注意：AudioManager 的构造函数中初始化了 SoundClient
        // 这需要系统中运行着 sound_play 节点
        AudioManager audio_mgr(nh);

        // 设置循环频率，例如每 5 秒测试一次 (0.2 Hz)
        ros::Rate loop_rate(1); 

        while (ros::ok()) {
            ROS_INFO("--------------------------------");
            ROS_INFO("Test Trigger: Playing CAMERA_ERR");

            // 4. 调用播放接口
            // 这里使用你在截图中确实存在的 camera_err.wav 对应的枚举
            audio_mgr.play(AudioType::NETWORK_ERR);

            // 处理回调（虽然这里主要是发送指令，但保留 spinOnce 是好习惯）
            ros::spinOnce();

            // 休眠直到下一个周期
            loop_rate.sleep();
        }
    }
    catch (const std::exception& e) {
        ROS_ERROR("Exception occurred: %s", e.what());
    }

    return 0;
}
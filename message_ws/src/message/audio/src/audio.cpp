#include "audio/audio.h"
#include "logger/logger.h"
#include <ros/package.h> // 必须包含，用于查找包路径

AudioManager::AudioManager(ros::NodeHandle& nh) : sc_(nh, "robotsound") {
    // 初始化路径映射
    initAudioPaths();
    InitSystemVolume(30);
    
}

AudioManager::~AudioManager() {
    // 析构逻辑
}

void AudioManager::initAudioPaths() {
    // 1. 获取当前功能包的绝对路径
    std::string pkg_path = ros::package::getPath("audio");
    
    if (pkg_path.empty()) {
        LOG_ERROR("Failed to find package 'audio'. Audio will not work.");
        return;
    }

    std::string sound_dir = pkg_path + "/sounds/";

    // 2. 绑定枚举到具体的文件名
    audio_map_[AudioType::NETWORK_CONNECTED]  = sound_dir + "network_connected.wav";

    audio_map_[AudioType::CAMERA_ERR]  = sound_dir + "camera_err.wav";
    audio_map_[AudioType::NETWORK_ERR]  = sound_dir + "network_err.wav";
    audio_map_[AudioType::RTK_ERR]     = sound_dir + "rtk_err.wav";
    audio_map_[AudioType::LIDAR_ERR]   = sound_dir + "lidar_err.wav";
    audio_map_[AudioType::RC_COMM_ERR]   = sound_dir + "rc_comm_err.wav";
    audio_map_[AudioType::MCU_COMM_ERR]   = sound_dir + "mcu_comm_err.wav";


    audio_map_[AudioType::IMU_DISCONNECTED]  = sound_dir + "imu_disconnected.wav";
    audio_map_[AudioType::LIDAR_DISCONNECTED]  = sound_dir + "lidar_disconnected.wav";

    audio_map_[AudioType::BATTERY_LOW] = sound_dir + "battery_low.wav";
    audio_map_[AudioType::TASK_START]  = sound_dir + "task_start.wav";
}

void AudioManager::play(AudioType type) {
    if (audio_map_.find(type) != audio_map_.end()) {
        std::string file_path = audio_map_[type];
        LOG_INFO("Playing audio: {}", file_path);
        
        std::string command = "aplay -q " + file_path + " &";
        
        // 执行命令
        int ret = std::system(command.c_str());
        
    } else {
        LOG_WARN("Audio type not defined in map!");
    }
}

void AudioManager::stop() {
    sc_.stopAll();
}

void AudioManager::InitSystemVolume(int volume_percent) {
    if (volume_percent < 0) volume_percent = 0;
    if (volume_percent > 150) volume_percent = 150;

    // > /dev/null 2>&1 用于屏蔽终端输出
    std::string cmd = "pactl set-sink-volume @DEFAULT_SINK@ " + std::to_string(volume_percent) + "% > /dev/null 2>&1";
    
    // 执行命令
    int ret = std::system(cmd.c_str());
    
    if (ret == 0) {
        LOG_INFO("[System] Volume initialized to {}% ", volume_percent);
    } else {
        LOG_ERROR("[System] Failed to set volume. Is PulseAudio running?");
    }
}

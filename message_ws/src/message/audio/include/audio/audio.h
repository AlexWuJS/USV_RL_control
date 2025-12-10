#ifndef AUDIO_SRC_AUDIO_H
#define AUDIO_SRC_AUDIO_H

#include <ros/ros.h>
#include <sound_play/sound_play.h>
#include <map>
#include <string>

// 1. 定义枚举类型，列出所有需要播报的情况
enum class AudioType {
    CAMERA_ERR,     // 相机故障
    RTK_ERR,        // RTK 定位故障
    LIDAR_ERR,      // 雷达故障
    BATTERY_LOW,    // 电量低
    TASK_START      // 任务开始
};

class AudioManager {
public:
    /**
     * @brief 构造函数
     * @param nh 传入节点的 NodeHandle
     */
    AudioManager(ros::NodeHandle& nh);
    ~AudioManager();

    /**
     * @brief 核心接口：根据枚举播放对应的语音
     * @param type 传入枚举变量 (e.g., AudioType::CAMERA_ERR)
     */
    void play(AudioType type);

    /**
     * @brief 停止所有正在播放的声音
     */
    void stop();

private:
    // sound_play 客户端
    sound_play::SoundClient sc_;
    
    // 用于存储枚举到文件路径的映射
    std::map<AudioType, std::string> audio_map_;

    // 初始化文件路径的方法
    void initAudioPaths();
};

#endif
#ifndef TCP_NETWORK_MONITOR_H_
#define TCP_NETWORK_MONITOR_H_


#include <ros/ros.h>

#include "audio/audio.h"      
#include "tcp/Tcp.h"     

// ==========================================
// 网络检测配置宏定义
// ==========================================
#define NET_TEST_IP        "223.5.5.5"   // 目标IP (阿里DNS)
#define NET_TEST_PORT      53            // 目标端口 (DNS)
#define NET_TIMEOUT_SEC    1             // 单次Socket连接超时时间 (秒)
#define NET_MAX_FAIL_COUNT 3             // 判定断网的连续失败次数阈值
#define NET_CHECK_INTERVAL 1             // 网络状态监测的时间间隔(秒)
/**
 * @brief 网络状态监控类
 * 负责定期检测网络连通性，并调用 AudioManager 播放对应语音
 */
class NetworkMonitor {
public:
    /**
     * @brief 构造函数
     * @param audio 传入 AudioManager 的引用 (依赖注入)
     */
    NetworkMonitor(AudioManager& audio);

    /**
     * @brief 析构函数 (默认即可)
     */
    ~NetworkMonitor() = default;

    /**
     * @brief 核心更新函数，供 ros::Timer 回调使用
     * @param event ROS定时器事件数据
     */
    void update(const ros::TimerEvent& event);

private:
    // --- 外部依赖 ---
    AudioManager& audio_mgr_;  // 持有音频管理器的引用

    // --- 内部状态 ---
    int fail_count_;           // 连续失败计数器
    bool is_online_;           // 当前记录的网络在线状态
    bool is_first_run_;        // 是否是首次运行标记
};

#endif // TCP_NETWORK_MONITOR_H_
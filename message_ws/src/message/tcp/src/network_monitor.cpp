#include "tcp/network_monitor.h" 
#include "tcp/Tcp.h"        

#include "logger/logger.h"
#include <audio/audio.h>


NetworkMonitor::NetworkMonitor(AudioManager& audio) 
    : audio_mgr_(audio), fail_count_(0), is_online_(false), is_first_run_(true) {}

void NetworkMonitor::update(const ros::TimerEvent& event) {

    bool pass = TcpClient::checkNetStatus(NET_TEST_IP, NET_TEST_PORT, NET_TIMEOUT_SEC);

    // ---------------------------------------------------------
    // 场景 A: 首次运行 (立即播报，忽略计数器)
    // ---------------------------------------------------------
    if (is_first_run_) {
        is_first_run_ = false; // 标记首次运行结束
        
        if (pass) {
            is_online_ = true;
            fail_count_ = 0;
            LOG_INFO("[NetMonitor] Boot Check: Network ONLINE.");
            audio_mgr_.play(AudioType::NETWORK_CONNECTED);
        } else {
            is_online_ = false;
            fail_count_ = NET_MAX_FAIL_COUNT; // 开机没网，直接把计数器拉满
            LOG_WARN("[NetMonitor] Boot Check: Network OFFLINE.");
            audio_mgr_.play(AudioType::NETWORK_ERR);
        }
        return; // 首次处理完直接返回，不走下面的逻辑
    }

    // ---------------------------------------------------------
    // 场景 B: 后续周期运行 (带去抖动逻辑)
    // ---------------------------------------------------------
    if (pass) {
        // --- 检测成功 ---
        fail_count_ = 0; // 只要成功一次，计数器清零
        
        // 只有状态从“离线”变成了“在线”才播报
        if (!is_online_) { 
            is_online_ = true;
            LOG_INFO("[NetMonitor] Status Changed: RECOVERED (Online)");
            audio_mgr_.play(AudioType::NETWORK_CONNECTED);
        }
    } else {
        // --- 检测失败 ---
        fail_count_++;
        LOG_WARN_FILE("[NetMonitor] Network Check Failed {} time, try again...", fail_count_);
        
        // 只有连续失败次数达到阈值，且当前认为是“在线”时，才播报断网
        if (fail_count_ >= NET_MAX_FAIL_COUNT) {
            if (is_online_) { 
                is_online_ = false;
                LOG_WARN("[NetMonitor] Status Changed: LOST (Offline)");
                audio_mgr_.play(AudioType::NETWORK_ERR);
            }
            // 锁定计数器，防止无限增加溢出 (虽然 int 很难溢出，但这是好习惯)
            fail_count_ = NET_MAX_FAIL_COUNT; 
        }
    }
}

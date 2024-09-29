#include <iostream>
using namespace std;
#include "ros/ros.h"
#include <vector>
#include <string>

bool checkTopic(const std::string& topic_name) {
    std::vector<ros::master::TopicInfo> topics;
    if (ros::master::getTopics(topics)) {
        for (const auto& topic : topics) {
            if (topic.name == topic_name) {
                return true;
            }
        }
    }
    return false;
}
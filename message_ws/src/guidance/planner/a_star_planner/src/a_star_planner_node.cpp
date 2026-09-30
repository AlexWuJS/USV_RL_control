#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/String.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>
#include <queue>
#include <unordered_set>
#include <cmath>
#include "a_star_planner.h"

AStarPlanner::AStarPlanner() : tf_listener_(tf_buffer_) {
    // 订阅地图、起点和目标点
    map_sub_ = nh_.subscribe("/prior_map", 1, &AStarPlanner::mapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &AStarPlanner::startCallback, this);  // 添加起点订阅
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AStarPlanner::goalCallback, this);
    goal_seq_sub_ = nh_.subscribe("/goal_sequence", 1, &AStarPlanner::goalSequenceCallback, this);

    // 发布路径、膨胀地图和规划状态
    path_pub_ = nh_.advertise<nav_msgs::Path>("/waypoints_path", 1);
    inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);
    status_pub_ = nh_.advertise<std_msgs::String>("/planner_status", 1, true);  // latched，方便后启动的订阅者拿到结果

    // 加载参数
    nh_.param("heuristic_weight", heuristic_weight_, 1.0);
    nh_.param("diagonal_movement", diagonal_movement_, true);
    nh_.param("max_iterations", max_iterations_, 100000);
    nh_.param("map_resolution", map_resolution_, 0.5);
    nh_.param("inflation_radius", inflation_radius_, 0.5);  // 膨胀半径，单位：米

    // 多目标串规划参数（launch 里 param 写在 <node> 内，走私有命名空间）
    ros::NodeHandle pnh("~");
    pnh.param<std::string>("on_goal_invalid", on_goal_invalid_, "abort");
    pnh.param<double>("min_seg_len", min_seg_len_, 1.0);
    pnh.param<bool>("fill_orientation", fill_orientation_, false);
    pnh.param<std::string>("robot_frame", robot_frame_, "camera_init");

    // 初始化起点标志
    start_received_ = false;

    ROS_INFO("A* Planner initialized with parameters:");
    ROS_INFO("  - Heuristic weight: %.2f", heuristic_weight_);
    ROS_INFO("  - Diagonal movement: %s", diagonal_movement_ ? "enabled" : "disabled");
    ROS_INFO("  - Max iterations: %d", max_iterations_);
    ROS_INFO("  - Map resolution: %.3f", map_resolution_);
    ROS_INFO("  - Inflation radius: %.3f meters", inflation_radius_);
    ROS_INFO("  - On goal invalid: %s", on_goal_invalid_.c_str());
    ROS_INFO("  - Min segment length: %.2f m", min_seg_len_);
    ROS_INFO("  - Fill orientation: %s", fill_orientation_ ? "true" : "false");
    ROS_INFO("  - Robot frame: %s", robot_frame_.c_str());
}

void AStarPlanner::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_pose_ = msg->pose.pose;
    start_received_ = true;
    ROS_INFO("Received new start position:");
    ROS_INFO("  - x: %.2f", start_pose_.position.x);
    ROS_INFO("  - y: %.2f", start_pose_.position.y);
}

void AStarPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    current_map_ = *msg;
    map_received_ = true;
    
    // 创建膨胀地图
    nav_msgs::OccupancyGrid inflated_map = current_map_;
    int inflation_cells = std::ceil(inflation_radius_ / current_map_.info.resolution);
    
    // 初始化膨胀地图数据
    std::vector<int8_t> inflated_data(current_map_.data.size(), 0);
    
    // 膨胀处理
    for (int y = 0; y < current_map_.info.height; ++y) {
        for (int x = 0; x < current_map_.info.width; ++x) {
            if (current_map_.data[y * current_map_.info.width + x] > 0) {  // 如果是障碍物
                // 在障碍物周围进行膨胀
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy) {
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx) {
                        int nx = x + dx;
                        int ny = y + dy;
                        if (nx >= 0 && nx < current_map_.info.width &&
                            ny >= 0 && ny < current_map_.info.height) {
                            // 使用欧氏距离计算膨胀
                            double dist = std::sqrt(dx*dx + dy*dy) * current_map_.info.resolution;
                            if (dist <= inflation_radius_) {
                                // 设置膨胀区域为100（完全占用）
                                inflated_data[ny * current_map_.info.width + nx] = 100;
                            }
                        }
                    }
                }
            }
        }
    }
    
    // 设置膨胀地图
    inflated_map.data = inflated_data;
    inflated_map.header = current_map_.header;
    inflated_map.header.stamp = ros::Time::now();
    inflated_map.info = current_map_.info;  // 确保信息一致
    
    // 发布膨胀地图
    inflated_map_pub_.publish(inflated_map);
    
    ROS_INFO("Received new map:");
    ROS_INFO("  - Map size: %d x %d", current_map_.info.width, current_map_.info.height);
    ROS_INFO("  - Map resolution: %.3f", current_map_.info.resolution);
    ROS_INFO("  - Map origin: (%.2f, %.2f)", 
             current_map_.info.origin.position.x,
             current_map_.info.origin.position.y);
    ROS_INFO("  - Inflation radius: %.3f meters", inflation_radius_);
    ROS_INFO("  - Inflation cells: %d", inflation_cells);
}

void AStarPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (!map_received_) {
        ROS_WARN("No map received yet");
        return;
    }

    // 起点：优先艇当前位置（TF map→robot_frame），/initialpose（重定位点）仅兜底
    geometry_msgs::Pose seg_start;
    if (getBoatPoseInMap(seg_start)) {
        ROS_INFO("Start position: boat TF pose");
    } else if (start_received_) {
        seg_start = start_pose_;
        ROS_WARN("Boat TF unavailable, start from /initialpose"
                 "（重定位点——可能不是艇当前位置）");
    } else {
        ROS_WARN("No boat TF and no /initialpose start received");
        return;
    }

    ROS_INFO("Start position (world coordinates):");
    ROS_INFO("  - x: %.2f", seg_start.position.x);
    ROS_INFO("  - y: %.2f", seg_start.position.y);

    ROS_INFO("Goal position (world coordinates):");
    ROS_INFO("  - x: %.2f", msg->pose.position.x);
    ROS_INFO("  - y: %.2f", msg->pose.position.y);

    std::vector<Node*> path;
    if (!planSegment(seg_start, msg->pose, path)) {
        publishStatus("ERROR: single goal unreachable");
    } else {
        ROS_INFO("Path found with %lu points", path.size());
        publishStatus("OK: single goal, " + std::to_string(path.size()) + " poses");
    }

    // 发布路径
    publishPath(path);

    // 清理内存
    freePath(path);
}

void AStarPlanner::goalSequenceCallback(const nav_msgs::Path::ConstPtr& msg) {
    if (!map_received_) {
        ROS_WARN("[goal_sequence] No map received yet");
        publishStatus("ERROR: no map");
        return;
    }
    if (msg->poses.empty()) {
        ROS_WARN("[goal_sequence] Empty goal sequence");
        return;
    }

    // 相邻航点去重：间距小于 min_seg_len_ 的合并
    std::vector<geometry_msgs::Pose> goals;
    for (const auto& ps : msg->poses) {
        if (!goals.empty()) {
            double dx = ps.pose.position.x - goals.back().position.x;
            double dy = ps.pose.position.y - goals.back().position.y;
            if (std::sqrt(dx * dx + dy * dy) < min_seg_len_) {
                ROS_INFO("[goal_sequence] Skip waypoint too close to previous: (%.2f, %.2f)",
                         ps.pose.position.x, ps.pose.position.y);
                continue;
            }
        }
        goals.push_back(ps.pose);
    }
    if (goals.empty()) {
        ROS_WARN("[goal_sequence] All waypoints merged, nothing to plan");
        publishStatus("ERROR: all waypoints merged");
        return;
    }

    // 第 1 段起点：优先 TF 取艇当前位姿，其次退回 /initialpose
    geometry_msgs::Pose seg_start;
    if (getBoatPoseInMap(seg_start)) {
        ROS_INFO("[goal_sequence] Segment 1 starts from boat TF pose");
    } else if (start_received_) {
        seg_start = start_pose_;
        ROS_INFO("[goal_sequence] Boat TF unavailable, start from /initialpose");
    } else {
        ROS_WARN("[goal_sequence] No boat TF and no /initialpose start received");
        publishStatus("ERROR: no start available");
        return;
    }

    // 逐段规划并拼接：第 i 段起点 = 上一段终点，保证整条路径连续
    std::vector<Node*> mission;
    for (size_t i = 0; i < goals.size(); ++i) {
        ROS_INFO("[goal_sequence] Planning segment %lu/%lu -> (%.2f, %.2f)",
                 i + 1, goals.size(), goals[i].position.x, goals[i].position.y);
        std::vector<Node*> seg;
        if (!planSegment(seg_start, goals[i], seg)) {
            freePath(seg);
            if (on_goal_invalid_ == "skip") {
                // 跳过该航点，下一段仍从上一个成功航点出发
                ROS_WARN("[goal_sequence] Waypoint %lu unreachable, skipped", i + 1);
                continue;
            }
            freePath(mission);
            publishStatus("ERROR: waypoint " + std::to_string(i + 1) + " unreachable (abort)");
            return;
        }
        if (!mission.empty() && !seg.empty() &&
            mission.back()->x == seg.front()->x && mission.back()->y == seg.front()->y) {
            delete seg.front();  // 相邻段连接点去重
            seg.erase(seg.begin());
        }
        mission.insert(mission.end(), seg.begin(), seg.end());
        seg_start = goals[i];
    }

    if (mission.empty()) {
        publishStatus("ERROR: all waypoints unreachable");
        return;
    }

    ROS_INFO("[goal_sequence] Mission planned: %lu goals, %lu poses",
             goals.size(), mission.size());
    publishStatus("OK: " + std::to_string(goals.size()) + " goals, " +
                  std::to_string(mission.size()) + " poses");
    publishPath(mission);
    freePath(mission);
}

bool AStarPlanner::getBoatPoseInMap(geometry_msgs::Pose& out) {
    try {
        geometry_msgs::TransformStamped tf_map_robot =
            tf_buffer_.lookupTransform("map", robot_frame_, ros::Time(0), ros::Duration(0.2));
        out.position.x = tf_map_robot.transform.translation.x;
        out.position.y = tf_map_robot.transform.translation.y;
        out.position.z = 0.0;
        out.orientation = tf_map_robot.transform.rotation;
        return true;
    } catch (tf2::TransformException& ex) {
        ROS_WARN_THROTTLE(1.0, "TF map -> %s unavailable: %s", robot_frame_.c_str(), ex.what());
        return false;
    }
}

bool AStarPlanner::planSegment(const geometry_msgs::Pose& start, const geometry_msgs::Pose& goal,
                               std::vector<Node*>& seg_out) {
    // 世界坐标 -> 栅格坐标（沿用既有约定：内部栅格按 map_resolution_ 划分）
    int start_x = (start.position.x - current_map_.info.origin.position.x) / map_resolution_;
    int start_y = (start.position.y - current_map_.info.origin.position.y) / map_resolution_;
    int goal_x = (goal.position.x - current_map_.info.origin.position.x) / map_resolution_;
    int goal_y = (goal.position.y - current_map_.info.origin.position.y) / map_resolution_;

    ROS_INFO("Grid coordinates (using resolution %.3f):", map_resolution_);
    ROS_INFO("  - Start: (%d, %d)", start_x, start_y);
    ROS_INFO("  - Goal: (%d, %d)", goal_x, goal_y);

    if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
        ROS_WARN("Start or goal point is out of map bounds or in obstacle");
        return false;
    }

    seg_out = findPath(start_x, start_y, goal_x, goal_y);
    if (seg_out.empty()) {
        ROS_WARN("No path found for segment");
        return false;
    }
    return true;
}

bool AStarPlanner::isValid(int x, int y) {
    // 将坐标转换回原始地图分辨率进行检查
    double resolution_scale = map_resolution_ / current_map_.info.resolution;
    int original_x = x * resolution_scale;
    int original_y = y * resolution_scale;
    
    // 检查是否在地图范围内
    if (original_x < 0 || original_x >= current_map_.info.width ||
        original_y < 0 || original_y >= current_map_.info.height) {
        return false;
    }
    
    // 检查是否为障碍物（任何非零值都视为障碍物）
    // 同时检查周围的栅格，确保不会穿过障碍物
    int check_radius = std::ceil(0.5 / current_map_.info.resolution);  // 检查半径
    for (int dy = -check_radius; dy <= check_radius; ++dy) {
        for (int dx = -check_radius; dx <= check_radius; ++dx) {
            int nx = original_x + dx;
            int ny = original_y + dy;
            if (nx >= 0 && nx < current_map_.info.width &&
                ny >= 0 && ny < current_map_.info.height) {
                if (current_map_.data[ny * current_map_.info.width + nx] > 0) {
                    return false;
                }
            }
        }
    }
    
    return true;
}

double AStarPlanner::calculateHeuristic(int x1, int y1, int x2, int y2) {
    return heuristic_weight_ * std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

std::vector<AStarPlanner::Node*> AStarPlanner::findPath(int start_x, int start_y, int goal_x, int goal_y) {
    ROS_INFO("Starting A* search from (%d, %d) to (%d, %d)", start_x, start_y, goal_x, goal_y);
    
    std::priority_queue<Node*, std::vector<Node*>, NodeCompare> open_set;
    std::unordered_set<Node*, NodeHash, NodeEqual> closed_set;
    
    Node* start_node = new Node(start_x, start_y);
    start_node->h_cost = calculateHeuristic(start_x, start_y, goal_x, goal_y);
    start_node->f_cost = start_node->h_cost;
    open_set.push(start_node);
    
    int iterations = 0;
    while (!open_set.empty() && iterations < max_iterations_) {
        Node* current = open_set.top();
        open_set.pop();
        
        if (current->x == goal_x && current->y == goal_y) {
            ROS_INFO("Path found after %d iterations", iterations);
            return reconstructPath(current);
        }
        
        closed_set.insert(current);
        
        // 检查相邻节点
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue;
                if (!diagonal_movement_ && dx != 0 && dy != 0) continue;
                
                int new_x = current->x + dx;
                int new_y = current->y + dy;
                
                if (!isValid(new_x, new_y)) continue;
                
                Node* neighbor = new Node(new_x, new_y);
                if (closed_set.find(neighbor) != closed_set.end()) {
                    delete neighbor;
                    continue;
                }
                
                double new_g_cost = current->g_cost + 
                    (dx != 0 && dy != 0 ? 1.414 : 1.0);  // 对角线距离为√2
                
                bool in_open_set = false;
                std::vector<Node*> temp_open;
                while (!open_set.empty()) {
                    Node* node = open_set.top();
                    open_set.pop();
                    if (node->x == new_x && node->y == new_y) {
                        if (new_g_cost < node->g_cost) {
                            node->g_cost = new_g_cost;
                            node->f_cost = node->g_cost + node->h_cost;
                            node->parent = current;
                        }
                        in_open_set = true;
                    }
                    temp_open.push_back(node);
                }
                
                for (Node* node : temp_open) {
                    open_set.push(node);
                }
                
                if (!in_open_set) {
                    neighbor->g_cost = new_g_cost;
                    neighbor->h_cost = calculateHeuristic(new_x, new_y, goal_x, goal_y);
                    neighbor->f_cost = neighbor->g_cost + neighbor->h_cost;
                    neighbor->parent = current;
                    open_set.push(neighbor);
                } else {
                    delete neighbor;
                }
            }
        }
        
        iterations++;
        if (iterations % 1000 == 0) {
            ROS_INFO("Search progress: %d iterations, open set size: %lu", 
                    iterations, open_set.size());
        }
    }
    
    ROS_WARN("No path found after %d iterations", iterations);
    return std::vector<Node*>();
}

std::vector<AStarPlanner::Node*> AStarPlanner::reconstructPath(Node* goal_node) {
    std::vector<Node*> path;
    Node* current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

void AStarPlanner::publishPath(const std::vector<Node*>& path) {
    if (path.empty()) return;

    // 清除之前的路径
    current_path_.poses.clear();

    current_path_.header = current_map_.header;
    current_path_.header.stamp = ros::Time::now();
    current_path_.header.frame_id = "map";

    double last_yaw = 0.0;
    for (size_t i = 0; i < path.size(); ++i) {
        geometry_msgs::PoseStamped pose;
        pose.header = current_path_.header;
        pose.pose.position.x = path[i]->x * map_resolution_ + current_map_.info.origin.position.x;
        pose.pose.position.y = path[i]->y * map_resolution_ + current_map_.info.origin.position.y;

        if (fill_orientation_) {
            // 朝向 = 指向下一个路径点的方向（末点沿用前一点方向）
            // 注意：填 orientation 会激活 huamo 的 is_turn 转弯分支，默认关闭
            double yaw = last_yaw;
            if (i + 1 < path.size()) {
                double dx = (path[i + 1]->x - path[i]->x) * map_resolution_;
                double dy = (path[i + 1]->y - path[i]->y) * map_resolution_;
                if (std::sqrt(dx * dx + dy * dy) > 1e-6) {
                    yaw = std::atan2(dy, dx);
                }
            }
            pose.pose.orientation.z = std::sin(yaw / 2.0);
            pose.pose.orientation.w = std::cos(yaw / 2.0);
            last_yaw = yaw;
        } else {
            pose.pose.orientation.w = 1.0;
        }

        current_path_.poses.push_back(pose);
    }

    path_pub_.publish(current_path_);
}

void AStarPlanner::publishStatus(const std::string& status) {
    std_msgs::String msg;
    msg.data = status;
    status_pub_.publish(msg);
    ROS_INFO("[planner_status] %s", status.c_str());
}

void AStarPlanner::freePath(std::vector<Node*>& path) {
    for (Node* node : path) {
        delete node;
    }
    path.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_planner_node");
    AStarPlanner planner;
    ros::spin();
    return 0;
} 
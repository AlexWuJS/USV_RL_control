#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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
    
    // 发布路径和膨胀地图
    path_pub_ = nh_.advertise<nav_msgs::Path>("/waypoints_path", 1);
    inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);
    
    // 加载参数
    nh_.param("heuristic_weight", heuristic_weight_, 1.0);
    nh_.param("diagonal_movement", diagonal_movement_, true);
    nh_.param("max_iterations", max_iterations_, 100000);
    nh_.param("map_resolution", map_resolution_, 0.5);
    nh_.param("inflation_radius", inflation_radius_, 0.5);  // 膨胀半径，单位：米
    
    // 初始化起点标志
    start_received_ = false;
    
    ROS_INFO("A* Planner initialized with parameters:");
    ROS_INFO("  - Heuristic weight: %.2f", heuristic_weight_);
    ROS_INFO("  - Diagonal movement: %s", diagonal_movement_ ? "enabled" : "disabled");
    ROS_INFO("  - Max iterations: %d", max_iterations_);
    ROS_INFO("  - Map resolution: %.3f", map_resolution_);
    ROS_INFO("  - Inflation radius: %.3f", inflation_radius_);
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
    
    if (!start_received_) {
        ROS_WARN("No start position received yet. Please set start position in RViz");
        return;
    }
    
    // 使用已保存的起点
    geometry_msgs::PoseStamped start_pose;
    start_pose.header = current_map_.header;
    start_pose.pose = start_pose_;
    
    ROS_INFO("Start position (world coordinates):");
    ROS_INFO("  - x: %.2f", start_pose.pose.position.x);
    ROS_INFO("  - y: %.2f", start_pose.pose.position.y);
    
    // 将起点和终点转换为栅格坐标，考虑分辨率缩放
    double resolution_scale = map_resolution_ / current_map_.info.resolution;
    int start_x = (start_pose.pose.position.x - current_map_.info.origin.position.x) / map_resolution_;
    int start_y = (start_pose.pose.position.y - current_map_.info.origin.position.y) / map_resolution_;
    int goal_x = (msg->pose.position.x - current_map_.info.origin.position.x) / map_resolution_;
    int goal_y = (msg->pose.position.y - current_map_.info.origin.position.y) / map_resolution_;
    
    ROS_INFO("Goal position (world coordinates):");
    ROS_INFO("  - x: %.2f", msg->pose.position.x);
    ROS_INFO("  - y: %.2f", msg->pose.position.y);
    
    ROS_INFO("Grid coordinates (using resolution %.3f):", map_resolution_);
    ROS_INFO("  - Start: (%d, %d)", start_x, start_y);
    ROS_INFO("  - Goal: (%d, %d)", goal_x, goal_y);
    
    // 检查起点和终点是否在地图范围内
    if (!isValid(start_x, start_y) || !isValid(goal_x, goal_y)) {
        ROS_WARN("Start or goal point is out of map bounds or in obstacle");
        return;
    }
    
    // 执行A星搜索
    std::vector<Node*> path = findPath(start_x, start_y, goal_x, goal_y);
    
    if (!path.empty()) {
        ROS_INFO("Path found with %lu points", path.size());
    } else {
        ROS_WARN("No path found");
    }
    
    // 发布路径
    publishPath(path);
    
    // 清理内存
    for (Node* node : path) {
        delete node;
    }
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
    
    for (const Node* node : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = current_path_.header;
        pose.pose.position.x = node->x * map_resolution_ + current_map_.info.origin.position.x;
        pose.pose.position.y = node->y * map_resolution_ + current_map_.info.origin.position.y;
        pose.pose.orientation.w = 1.0;
        current_path_.poses.push_back(pose);
    }
    
    path_pub_.publish(current_path_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "a_star_planner_node");
    AStarPlanner planner;
    ros::spin();
    return 0;
} 
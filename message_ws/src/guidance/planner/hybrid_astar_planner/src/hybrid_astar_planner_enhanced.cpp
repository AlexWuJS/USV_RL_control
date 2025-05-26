#include <ros/ros.h>
#include "hybrid_astar_planner.h"
#include "hybrid_astar_searcher/hybrid_astar.h"
#include "hybrid_astar_searcher/ReedsSheppPath.h"
#include "hybrid_astar_searcher/smooth.h"
#include "hybrid_astar_searcher/calculate_heuristic.h"
#include "hybrid_astar_searcher/dynamicvoronoi.h"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <chrono>
#include <memory>

class Visualizer {
public:
    Visualizer(ros::NodeHandle& nh) {
        path_points_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/path_points", 1);
        vehicle_boxes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/vehicle_boxes", 1);
    }

    void publishPathPoint(const Eigen::Vector2d& point, int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_points";
        marker.id = id;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = point.x();
        marker.pose.position.y = point.y();
        marker.pose.position.z = 0.1;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        path_points_pub_.publish(marker_array);
    }

    void publishVehicleBoxes(const planning::Vec3d& pose, int id) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "vehicle_boxes";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        
        // 设置车辆尺寸（与hybridAstar_lbfgsSmooth保持一致）
        marker.scale.x = 4.933;  // Vehicle length
        marker.scale.y = 2.11;   // Vehicle width
        marker.scale.z = 1.0;    // Vehicle height
        
        // 设置颜色 - 微量蓝色（与hybridAstar_lbfgsSmooth保持一致）
        marker.color.a = 0.08;  // 非常透明
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        
        // 考虑车辆中心偏移（与hybridAstar_lbfgsSmooth保持一致）
        double centerX = pose.x() + 1.45 * std::cos(pose.z());
        double centerY = pose.y() + 1.45 * std::sin(pose.z());
        
        marker.pose.position.x = centerX;
        marker.pose.position.y = centerY;
        marker.pose.position.z = 0.1;
        
        // 设置车辆方向（偏航角）
        tf2::Quaternion q;
        q.setRPY(0, 0, pose.z());
        marker.pose.orientation = tf2::toMsg(q);
        
        visualization_msgs::MarkerArray marker_array;
        marker_array.markers.push_back(marker);
        vehicle_boxes_pub_.publish(marker_array);
    }
    
    // 清除所有路径点和车辆边界框
    void clearAllVisualizations() {
        // 清除路径点
        visualization_msgs::MarkerArray empty_points;
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "path_points";
        marker.id = 0;
        marker.action = visualization_msgs::Marker::DELETEALL;
        empty_points.markers.push_back(marker);
        path_points_pub_.publish(empty_points);
        
        // 清除车辆边界框
        visualization_msgs::MarkerArray empty_boxes;
        marker.ns = "vehicle_boxes";
        empty_boxes.markers.push_back(marker);
        vehicle_boxes_pub_.publish(empty_boxes);
    }

private:
    ros::Publisher path_points_pub_;
    ros::Publisher vehicle_boxes_pub_;
};

HybridAStarPlanner::HybridAStarPlanner() : tf_listener_(tf_buffer_) {
    // Initialize parameters
    nh_.param("heuristic_weight", heuristic_weight_, 1.0);
    nh_.param("inflation_radius", inflation_radius_, 0.5);
    nh_.param("map_resolution", map_resolution_, 0.05);
    nh_.param("vehicle_length", vehicle_length_, 4.5);
    nh_.param("vehicle_width", vehicle_width_, 2.0);
    nh_.param("min_turning_radius", min_turning_radius_, 5.0);
    nh_.param("steering_angle", steering_angle_, 0.5);
    nh_.param("steering_angle_discrete_num", steering_angle_discrete_num_, 5);
    nh_.param("segment_length", segment_length_, 1.0);
    nh_.param("max_iterations", max_iterations_, 10000);
    nh_.param("visualize_vehicle", visualize_vehicle_, true);
    nh_.param("max_planning_attempts", max_planning_attempts_, 3);
    nh_.param("planning_timeout", planning_timeout_, 5.0);
    
    // Initialize smoothing parameters
    XmlRpc::XmlRpcValue smoother_params;
    if (nh_.getParam("smoother_params", smoother_params)) {
        if (smoother_params.hasMember("alpha")) {
            smoother_alpha_ = static_cast<double>(smoother_params["alpha"]);
        }
        if (smoother_params.hasMember("beta")) {
            smoother_beta_ = static_cast<double>(smoother_params["beta"]);
        }
        if (smoother_params.hasMember("max_iterations")) {
            smoother_max_iterations_ = static_cast<int>(smoother_params["max_iterations"]);
        }
        if (smoother_params.hasMember("weight_collision")) {
            smoother_weight_collision_ = static_cast<double>(smoother_params["weight_collision"]);
        }
        if (smoother_params.hasMember("weight_curvature")) {
            smoother_weight_curvature_ = static_cast<double>(smoother_params["weight_curvature"]);
        }
        if (smoother_params.hasMember("weight_smoothness")) {
            smoother_weight_smoothness_ = static_cast<double>(smoother_params["weight_smoothness"]);
        }
    }

    // Initialize subscribers and publishers
    map_sub_ = nh_.subscribe("/map", 1, &HybridAStarPlanner::mapCallback, this);
    grid_map_sub_ = nh_.subscribe("/grid_map", 1, &HybridAStarPlanner::gridMapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &HybridAStarPlanner::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &HybridAStarPlanner::goalCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/hybrid_astar_path", 1);
    inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);
    rs_path_pub_ = nh_.advertise<visualization_msgs::Marker>("/rs_path_marker", 1);
    
    // Initialize visualizer
    visualizer_ = std::make_shared<Visualizer>(nh_);

    // Initialize an empty map with a large range to ensure no boundary issues
    grid_map::GridMap empty_map;
    empty_map.setGeometry(grid_map::Length(500.0, 500.0), 0.05, grid_map::Position(0.0, 0.0));
    empty_map.add("elevation", 0.0);

    try {
        // Initialize hybrid A* searcher and smoother
        hybrid_astar_searcher_ = std::make_shared<planning::HybridAstar>(empty_map);
        smoother_ = std::make_shared<planning::Smoother>();

        ROS_INFO("Hybrid A* Planner initialized with parameters:");
        ROS_INFO("  - Heuristic weight: %.2f", heuristic_weight_);
        ROS_INFO("  - Inflation radius: %.3f", inflation_radius_);
        ROS_INFO("  - Map resolution: %.3f", map_resolution_);
        ROS_INFO("  - Vehicle length: %.2f", vehicle_length_);
        ROS_INFO("  - Vehicle width: %.2f", vehicle_width_);
        ROS_INFO("  - Min turning radius: %.2f", min_turning_radius_);
        ROS_INFO("  - Steering angle: %.2f", steering_angle_);
        ROS_INFO("  - Steering angle discrete num: %d", steering_angle_discrete_num_);
        ROS_INFO("  - Segment length: %.2f", segment_length_);
        ROS_INFO("  - Max iterations: %d", max_iterations_);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in HybridAStarPlanner constructor: %s", e.what());
    }
}

void HybridAStarPlanner::gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg) {
    try {
        ROS_INFO("Received grid map message");
        
        // Convert grid map message to grid map
        grid_map::GridMap grid_map;
        grid_map::GridMapRosConverter::fromMessage(*msg, grid_map);
        
        // Update grid map and searcher
        current_grid_map_ = grid_map;
        
        // Create new hybrid A* searcher
        hybrid_astar_searcher_ = std::make_shared<planning::HybridAstar>(current_grid_map_);
        
        // Print grid map information
        grid_map::Length length = current_grid_map_.getLength();
        grid_map::Position position = current_grid_map_.getPosition();
        double x_min = position.x() - length(0) / 2.0;
        double x_max = position.x() + length(0) / 2.0;
        double y_min = position.y() - length(1) / 2.0;
        double y_max = position.y() + length(1) / 2.0;
        
        ROS_INFO("Grid map received with bounds: x_min: %.2f, x_max: %.2f, y_min: %.2f, y_max: %.2f",
                 x_min, x_max, y_min, y_max);
        
        map_received_ = true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in gridMapCallback: %s", e.what());
    }
}

// 实现边界检查函数 - 完全移除边界限制
bool HybridAStarPlanner::isWithinGridMapBounds(const grid_map::Position& position) {
    // 完全移除边界限制，始终返回true
    return true;
}

bool HybridAStarPlanner::safelySetElevation(const grid_map::Position& position, float value) {
    grid_map::Index index;
    try {
        // 直接调用getIndex而不检查边界
        if (current_grid_map_.getIndex(position, index)) {
            current_grid_map_.at("elevation", index) = value;
            return true;
        }
    } catch (const std::out_of_range& e) {
        // 忽略超出范围的异常
        ROS_DEBUG_THROTTLE(5.0, "Position (%.2f, %.2f) is outside grid map data range", position.x(), position.y());
    } catch (const std::exception& e) {
        ROS_DEBUG_THROTTLE(5.0, "Error accessing grid map: %s", e.what());
    }
    return false;
}

void HybridAStarPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    try {
        ROS_INFO("Received occupancy grid map message");
        current_map_ = *msg;
        
        // 打印地图尺寸信息
        ROS_INFO("Map dimensions: width=%d, height=%d, resolution=%.3f", 
                 current_map_.info.width, current_map_.info.height, current_map_.info.resolution);
        ROS_INFO("Map origin: x=%.3f, y=%.3f, z=%.3f", 
                 current_map_.info.origin.position.x,
                 current_map_.info.origin.position.y,
                 current_map_.info.origin.position.z);
                 
        // 计算地图真实大小（米）
        double map_width_m = current_map_.info.width * current_map_.info.resolution;
        double map_height_m = current_map_.info.height * current_map_.info.resolution;
        ROS_INFO("Map size in meters: %.2f x %.2f", map_width_m, map_height_m);
        
        // 增加地图扩展系数，确保地图足够大
        double map_extension_factor = 1.5;  // 扩大1.5倍
        double extended_width = map_width_m * map_extension_factor;
        double extended_height = map_height_m * map_extension_factor;
        
        // Create inflated map
        nav_msgs::OccupancyGrid inflated_map = current_map_;
        int inflation_cells = static_cast<int>(inflation_radius_ / current_map_.info.resolution);
        
        ROS_INFO("Creating inflated map with inflation radius: %.2f", inflation_radius_);
        
        for (int i = 0; i < current_map_.info.width; ++i) {
            for (int j = 0; j < current_map_.info.height; ++j) {
                if (current_map_.data[j * current_map_.info.width + i] > 50) {
                    // Inflate obstacles
                    for (int di = -inflation_cells; di <= inflation_cells; ++di) {
                        for (int dj = -inflation_cells; dj <= inflation_cells; ++dj) {
                            int ni = i + di;
                            int nj = j + dj;
                            if (ni >= 0 && ni < current_map_.info.width &&
                                nj >= 0 && nj < current_map_.info.height) {
                                int index = nj * current_map_.info.width + ni;
                                if (inflated_map.data[index] == 0) {
                                    inflated_map.data[index] = 50;
                                }
                            }
                        }
                    }
                }
            }
        }
        
        ROS_INFO("Publishing inflated map");
        inflated_map_pub_.publish(inflated_map);
        
        // 重新创建grid_map，设置大小为扩展后的地图大小，保留原点位置
        current_grid_map_ = grid_map::GridMap();
        
        // 设置grid_map的几何属性，使用扩展后的尺寸但保持原点位置
        current_grid_map_.setGeometry(grid_map::Length(extended_width, extended_height),
                            current_map_.info.resolution,
                            grid_map::Position(current_map_.info.origin.position.x + map_width_m/2, 
                                              current_map_.info.origin.position.y + map_height_m/2));
        
        // 打印grid_map信息
        grid_map::Length length = current_grid_map_.getLength();
        grid_map::Position position = current_grid_map_.getPosition();
        double x_min = position.x() - length(0) / 2.0;
        double x_max = position.x() + length(0) / 2.0;
        double y_min = position.y() - length(1) / 2.0;
        double y_max = position.y() + length(1) / 2.0;
        ROS_INFO("Grid map geometry: length=(%.2f, %.2f), position=(%.2f, %.2f)", 
                 length(0), length(1), position.x(), position.y());
        ROS_INFO("Grid map bounds: x=[%.2f, %.2f], y=[%.2f, %.2f]", x_min, x_max, y_min, y_max);
        
        // 更新地图边界 - 使用极大值，完全移除限制
        map_x_min_ = -std::numeric_limits<double>::max();
        map_y_min_ = -std::numeric_limits<double>::max();
        map_x_max_ = std::numeric_limits<double>::max();
        map_y_max_ = std::numeric_limits<double>::max();
        
        ROS_INFO("Map boundary restrictions removed, allowing planning in unlimited area");
        
        ROS_INFO("Adding elevation layer");
        current_grid_map_.add("elevation", 0.0);
        
        ROS_INFO("Filling grid map with obstacle data");
        int warnings_count = 0;
        int max_warnings = 10; // 最多显示10个警告
        
        for (int i = 0; i < current_map_.info.width; ++i) {
            for (int j = 0; j < current_map_.info.height; ++j) {
                int index = j * current_map_.info.width + i;
                if (inflated_map.data[index] > 50) {
                    try {
                        // 计算实际位置，确保坐标系转换正确
                        double x = current_map_.info.origin.position.x + (i + 0.5) * current_map_.info.resolution;
                        double y = current_map_.info.origin.position.y + (j + 0.5) * current_map_.info.resolution;
                        
                        grid_map::Position position(x, y);
                        // 不检查边界，直接设置elevation
                        bool success = safelySetElevation(position, 1.0);
                        if (!success && warnings_count < max_warnings) {
                            ROS_WARN_THROTTLE(1.0, "Failed to set elevation at (%.2f, %.2f)", x, y);
                            warnings_count++;
                        }
                    } catch (const std::exception& e) {
                        if (warnings_count < max_warnings) {
                            ROS_ERROR_THROTTLE(1.0, "Error accessing grid map at (%d, %d): %s", i, j, e.what());
                            warnings_count++;
                        }
                    }
                }
            }
        }
        
        ROS_INFO("Creating new hybrid A* searcher");
        // 创建新的hybrid A* searcher实例
        hybrid_astar_searcher_ = std::make_shared<planning::HybridAstar>(current_grid_map_);
        
        // 打印最终的地图信息
        ROS_INFO("Map processing completed successfully");
        ROS_INFO("  - Map size: %d x %d", current_map_.info.width, current_map_.info.height);
        ROS_INFO("  - Map resolution: %.3f", current_map_.info.resolution);
        ROS_INFO("  - Map origin: (%.2f, %.2f)", current_map_.info.origin.position.x, current_map_.info.origin.position.y);
        ROS_INFO("  - Grid map size: %.2f x %.2f", length(0), length(1));
        
        map_received_ = true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in mapCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in mapCallback");
    }
}

void HybridAStarPlanner::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    try {
        start_pose_ = msg->pose.pose;
        
        // 移除起点边界检查，直接接受任何位置的起点
        ROS_INFO("Received start pose: (%.2f, %.2f)", start_pose_.position.x, start_pose_.position.y);
        start_received_ = true;
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in startCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in startCallback");
    }
}

// 清除之前的路径
void HybridAStarPlanner::clearPreviousPath() {
    // 清除可视化元素
    visualizer_->clearAllVisualizations();
    
    // 清除Reed-Shepp路径标记
    visualization_msgs::Marker empty_marker;
    empty_marker.header.frame_id = "map";
    empty_marker.header.stamp = ros::Time::now();
    empty_marker.ns = "rs_path";
    empty_marker.id = 1;
    empty_marker.action = visualization_msgs::Marker::DELETE;
    rs_path_pub_.publish(empty_marker);
    
    // 发布空路径消息，清除已发布的路径
    nav_msgs::Path empty_path;
    empty_path.header.frame_id = "map";
    empty_path.header.stamp = ros::Time::now();
    path_pub_.publish(empty_path);
    
    ROS_INFO("Previous path cleared");
}

void HybridAStarPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    try {
        if (!map_received_ || !start_received_) {
            ROS_WARN("Waiting for map and start pose...");
            return;
        }

        // 清除之前的路径
        clearPreviousPath();

        geometry_msgs::Pose goal_pose = msg->pose;
        
        // 移除终点边界检查，直接接受任何位置的终点
        ROS_INFO("Received goal pose: (%.2f, %.2f)", goal_pose.position.x, goal_pose.position.y);

        // 创建起点和终点向量
        planning::Vec3d start(start_pose_.position.x, start_pose_.position.y, 
                             tf::getYaw(start_pose_.orientation));
        planning::Vec3d goal(goal_pose.position.x, goal_pose.position.y,
                            tf::getYaw(goal_pose.orientation));

        ROS_INFO("Planning path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", 
                 start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());
                 
        // 输出起点和终点信息
        ROS_INFO("Start position: (%.2f, %.2f)", start.x(), start.y());
        ROS_INFO("Goal position: (%.2f, %.2f)", goal.x(), goal.y());

        // 多次尝试规划，增加成功率
        planning::HybridAstarResult result;
        bool planning_success = false;
        int attempt = 0;
        
        while (!planning_success && attempt < max_planning_attempts_) {
            ROS_INFO("Planning attempt %d/%d", attempt+1, max_planning_attempts_);
            
            auto hybrid_astar_start = std::chrono::high_resolution_clock::now();
            planning_success = hybrid_astar_searcher_->plan(start, goal, result);
            auto hybrid_astar_end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
            
            if (planning_success) {
                ROS_INFO("Hybrid A* planning time: %.2f ms", hybrid_astar_use_time.count() * 1000);
                
                // Convert result to Vec3d path
                std::vector<planning::Vec3d> path;
                for (size_t i = 0; i < result.x.size(); ++i) {
                    path.emplace_back(result.x[i], result.y[i], result.theta[i]);
                }

                ROS_INFO("Found path with %zu points", path.size());

                // Visualize Reed-Shepp path
                visualization_msgs::Marker marker;
                marker.header.frame_id = "map";
                marker.header.stamp = ros::Time::now();
                marker.ns = "rs_path";
                marker.id = 1;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.orientation.w = 1.0;
                
                marker.type = visualization_msgs::Marker::LINE_STRIP;
                marker.scale.x = 0.05; // Line width

                // Set marker color and opacity
                marker.color.a = 1.0; // Fully opaque
                marker.color.r = 0.0; // Blue color
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                
                for (size_t i = 0; i < result.x.size(); ++i) {
                    geometry_msgs::Point point;
                    point.x = result.x[i];
                    point.y = result.y[i];
                    point.z = 0.1;
                    marker.points.push_back(point);
                    
                    // Visualize path points and vehicle boxes if enabled
                    if (visualize_vehicle_) {
                        visualizer_->publishPathPoint({result.x[i], result.y[i]}, i);
                        visualizer_->publishVehicleBoxes({result.x[i], result.y[i], result.theta[i]}, i);
                    }
                }
                rs_path_pub_.publish(marker);

                // 发布路径
                publishPath(path);
                ROS_INFO("Path published!");
                
                // 选择是否执行路径平滑（可能导致崩溃）
                bool perform_smoothing = false;
                
                if (perform_smoothing) {
                    try {
                        // 创建Voronoi图
                        DynamicVoronoi voronoi;
                        
                        // 使用智能指针安全管理内存
                        using BoolRowPtr = std::unique_ptr<bool[]>;
                        using BoolMapPtr = std::unique_ptr<BoolRowPtr[]>;
                        
                        // 创建二维布尔数组
                        BoolMapPtr binMap(new BoolRowPtr[current_map_.info.width]);
                        for (unsigned int x = 0; x < current_map_.info.width; x++) {
                            binMap[x] = BoolRowPtr(new bool[current_map_.info.height]);
                            for (unsigned int y = 0; y < current_map_.info.height; y++) {
                                int index = y * current_map_.info.width + x;
                                // 标记占用栅格为true（障碍物）
                                binMap[x][y] = (current_map_.data[index] > 50);
                            }
                        }
                        
                        // 获取原始指针数组以供Voronoi库使用
                        std::vector<bool*> rowPtrs(current_map_.info.width);
                        for (unsigned int x = 0; x < current_map_.info.width; x++) {
                            rowPtrs[x] = binMap[x].get();
                        }
                        
                        // 初始化和更新Voronoi图
                        voronoi.initializeMap(current_map_.info.width, current_map_.info.height, rowPtrs.data());
                        ROS_INFO("Voronoi map initialization complete, starting update");
                        voronoi.update();
                        ROS_INFO("Voronoi map update complete");
                        
                        // 路径平滑计时
                        ROS_INFO("Starting path smoothing");
                        auto smooth_path_start = std::chrono::high_resolution_clock::now();
                        smoother_->smoothPath(voronoi, path);
                        auto smooth_path_end = std::chrono::high_resolution_clock::now();
                        std::chrono::duration<double> smooth_path_use_time = smooth_path_end - smooth_path_start;
                        ROS_INFO("Path smoothing time: %.2f ms", smooth_path_use_time.count() * 1000);
                        ROS_INFO("Path smoothing complete, publishing smoothed path");
                        
                        // 智能指针会自动清理内存，不需要手动删除
                        
                        // 发布平滑后的路径
                        publishPath(path);
                        ROS_INFO("Smoothed path published!");
                    } catch (const std::exception& e) {
                        ROS_ERROR("Error processing Voronoi diagram: %s", e.what());
                        // 智能指针会自动清理内存，不需要手动删除
                    }
                } else {
                    ROS_INFO("Path smoothing disabled for stability. Using raw path.");
                }
                break;
            } else {
                ROS_WARN("Planning attempt %d failed, elapsed time: %.2f ms", 
                         attempt+1, hybrid_astar_use_time.count() * 1000);
                attempt++;
                // 短暂等待后重试
                ros::Duration(0.5).sleep();
            }
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in goalCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in goalCallback");
    }
}

void HybridAStarPlanner::publishPath(const std::vector<planning::Vec3d>& path) {
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    
    for (const auto& point : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        
        tf2::Quaternion q;
        q.setRPY(0, 0, point.z());
        pose.pose.orientation = tf2::toMsg(q);
        
        path_msg.poses.push_back(pose);
    }
    
    path_pub_.publish(path_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "hybrid_astar_planner");
    HybridAStarPlanner planner;
    ros::spin();
    return 0;
} 
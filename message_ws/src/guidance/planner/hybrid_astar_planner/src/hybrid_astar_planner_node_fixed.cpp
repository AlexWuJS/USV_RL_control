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

    // Initialize subscribers and publishers
    map_sub_ = nh_.subscribe("/map", 1, &HybridAStarPlanner::mapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &HybridAStarPlanner::startCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &HybridAStarPlanner::goalCallback, this);
    path_pub_ = nh_.advertise<nav_msgs::Path>("/hybrid_astar_path", 1);
    inflated_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/inflated_map", 1);

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

void HybridAStarPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    try {
        ROS_INFO("Received map message");
        current_map_ = *msg;
        map_received_ = true;
        
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
        
        // Update hybrid A* searcher map
        ROS_INFO("Creating grid map");
        
        // Calculate map dimensions and position
        double map_length_x = current_map_.info.width * current_map_.info.resolution;
        double map_length_y = current_map_.info.height * current_map_.info.resolution;
        
        // Use map origin as grid_map origin
        grid_map::Position map_origin(current_map_.info.origin.position.x,
                                    current_map_.info.origin.position.y);
        
        ROS_INFO("Map dimensions: %.2f x %.2f", map_length_x, map_length_y);
        ROS_INFO("Map origin: (%.2f, %.2f)", map_origin.x(), map_origin.y());
        
        // Recreate grid_map
        current_grid_map_ = grid_map::GridMap();
        
        // Set grid_map geometry
        current_grid_map_.setGeometry(grid_map::Length(map_length_x, map_length_y),
                            current_map_.info.resolution,
                            map_origin);
        
        ROS_INFO("Adding elevation layer");
        current_grid_map_.add("elevation", 0.0);
        
        ROS_INFO("Filling grid map with obstacle data");
        for (int i = 0; i < current_map_.info.width; ++i) {
            for (int j = 0; j < current_map_.info.height; ++j) {
                int index = j * current_map_.info.width + i;
                if (inflated_map.data[index] > 50) {
                    try {
                        // Calculate actual position
                        double x = current_map_.info.origin.position.x + (i + 0.5) * current_map_.info.resolution;
                        double y = current_map_.info.origin.position.y + (j + 0.5) * current_map_.info.resolution;
                        
                        // Get index in grid_map
                        grid_map::Position position(x, y);
                        grid_map::Index grid_index;
                        if (current_grid_map_.getIndex(position, grid_index)) {
                            current_grid_map_.at("elevation", grid_index) = 1.0;
                        } else {
                            ROS_WARN("Position (%.2f, %.2f) is outside grid map bounds", x, y);
                        }
                    } catch (const std::exception& e) {
                        ROS_ERROR("Error accessing grid map at (%d, %d): %s", i, j, e.what());
                    }
                }
            }
        }
        
        ROS_INFO("Creating new hybrid A* searcher");
        // Recreate hybrid A* searcher
        hybrid_astar_searcher_ = std::make_shared<planning::HybridAstar>(current_grid_map_);
        
        // Print grid map information
        grid_map::Length length = current_grid_map_.getLength();
        grid_map::Position position = current_grid_map_.getPosition();
        double x_min = position.x() - length(0) / 2.0;
        double x_max = position.x() + length(0) / 2.0;
        double y_min = position.y() - length(1) / 2.0;
        double y_max = position.y() + length(1) / 2.0;
        
        ROS_INFO("Grid map bounds: x_min: %.2f, x_max: %.2f, y_min: %.2f, y_max: %.2f",
                 x_min, x_max, y_min, y_max);
        
        ROS_INFO("Map processing completed successfully");
        ROS_INFO("  - Map size: %d x %d", current_map_.info.width, current_map_.info.height);
        ROS_INFO("  - Map resolution: %.3f", current_map_.info.resolution);
        ROS_INFO("  - Map origin: (%.2f, %.2f)", 
                 current_map_.info.origin.position.x,
                 current_map_.info.origin.position.y);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in mapCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in mapCallback");
    }
}

void HybridAStarPlanner::startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    try {
        start_pose_ = msg->pose.pose;
        
        // Check if start point is within map bounds
        if (map_received_) {
            // Get map bounds
            grid_map::Length length = current_grid_map_.getLength();
            grid_map::Position position = current_grid_map_.getPosition();
            double x_min = position.x() - length(0) / 2.0;
            double x_max = position.x() + length(0) / 2.0;
            double y_min = position.y() - length(1) / 2.0;
            double y_max = position.y() + length(1) / 2.0;
            
            // Log boundary information
            ROS_INFO("Map bounds: x_min: %.2f, x_max: %.2f, y_min: %.2f, y_max: %.2f",
                     x_min, x_max, y_min, y_max);
                     
            // Check if start point is within bounds, if not, issue warning but don't terminate
            if (start_pose_.position.x < x_min || start_pose_.position.x > x_max ||
                start_pose_.position.y < y_min || start_pose_.position.y > y_max) {
                ROS_WARN("Start point (%.2f, %.2f) is outside map bounds! Using closest point inside bounds.",
                         start_pose_.position.x, start_pose_.position.y);
                
                // Limit start position to map bounds
                start_pose_.position.x = std::max(x_min + 0.1, std::min(x_max - 0.1, start_pose_.position.x));
                start_pose_.position.y = std::max(y_min + 0.1, std::min(y_max - 0.1, start_pose_.position.y));
                
                ROS_INFO("Adjusted start point: (%.2f, %.2f)", start_pose_.position.x, start_pose_.position.y);
            }
        }
        
        start_received_ = true;
        ROS_INFO("Received start pose: (%.2f, %.2f)", start_pose_.position.x, start_pose_.position.y);
    } catch (const std::exception& e) {
        ROS_ERROR("Exception in startCallback: %s", e.what());
    } catch (...) {
        ROS_ERROR("Unknown exception in startCallback");
    }
}

void HybridAStarPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    try {
        if (!map_received_ || !start_received_) {
            ROS_WARN("Waiting for map and start pose...");
            return;
        }

        geometry_msgs::Pose goal_pose = msg->pose;
        
        // Check if goal point is within map bounds
        if (map_received_) {
            // Get map bounds
            grid_map::Length length = current_grid_map_.getLength();
            grid_map::Position position = current_grid_map_.getPosition();
            double x_min = position.x() - length(0) / 2.0;
            double x_max = position.x() + length(0) / 2.0;
            double y_min = position.y() - length(1) / 2.0;
            double y_max = position.y() + length(1) / 2.0;
            
            // Check if goal point is within bounds, if not, issue warning but don't terminate
            if (goal_pose.position.x < x_min || goal_pose.position.x > x_max ||
                goal_pose.position.y < y_min || goal_pose.position.y > y_max) {
                ROS_WARN("Goal point (%.2f, %.2f) is outside map bounds! Using closest point inside bounds.",
                         goal_pose.position.x, goal_pose.position.y);
                
                // Limit goal position to map bounds
                goal_pose.position.x = std::max(x_min + 0.1, std::min(x_max - 0.1, goal_pose.position.x));
                goal_pose.position.y = std::max(y_min + 0.1, std::min(y_max - 0.1, goal_pose.position.y));
                
                ROS_INFO("Adjusted goal point: (%.2f, %.2f)", goal_pose.position.x, goal_pose.position.y);
            }
        }

        // Create start and goal nodes
        planning::Vec3d start(start_pose_.position.x, start_pose_.position.y, 
                             tf::getYaw(start_pose_.orientation));
        planning::Vec3d goal(goal_pose.position.x, goal_pose.position.y,
                            tf::getYaw(goal_pose.orientation));

        ROS_INFO("Planning path from (%.2f, %.2f, %.2f) to (%.2f, %.2f, %.2f)", 
                 start.x(), start.y(), start.z(), goal.x(), goal.y(), goal.z());

        // Execute hybrid A* search
        planning::HybridAstarResult result;
        if (hybrid_astar_searcher_->plan(start, goal, result)) {
            // Convert result to Vec3d path
            std::vector<planning::Vec3d> path;
            for (size_t i = 0; i < result.x.size(); ++i) {
                path.emplace_back(result.x[i], result.y[i], result.theta[i]);
            }

            ROS_INFO("Found path with %zu points", path.size());

            // Create Voronoi diagram
            DynamicVoronoi voronoi;
            
            // Initialize Voronoi map, allocate memory for each cell
            bool** binMap = new bool*[current_map_.info.width];
            for (unsigned int x = 0; x < current_map_.info.width; x++) {
                binMap[x] = new bool[current_map_.info.height];
                for (unsigned int y = 0; y < current_map_.info.height; y++) {
                    int index = y * current_map_.info.width + x;
                    // Mark occupied cells as true (obstacles)
                    binMap[x][y] = (current_map_.data[index] > 50);
                }
            }
            
            try {
                // Initialize Voronoi diagram with binary map
                voronoi.initializeMap(current_map_.info.width, current_map_.info.height, binMap);
                ROS_INFO("Voronoi map initialization complete, starting update");
                voronoi.update();
                ROS_INFO("Voronoi map update complete");
                
                // Path smoothing
                ROS_INFO("Starting path smoothing");
                smoother_->smoothPath(voronoi, path);
                ROS_INFO("Path smoothing complete, publishing path");
                
                // Clean up memory
                for (unsigned int x = 0; x < current_map_.info.width; x++) {
                    delete[] binMap[x];
                }
                delete[] binMap;
            } catch (const std::exception& e) {
                ROS_ERROR("Error processing Voronoi diagram: %s", e.what());
                
                // Clean up memory
                for (unsigned int x = 0; x < current_map_.info.width; x++) {
                    delete[] binMap[x];
                }
                delete[] binMap;
                
                // Skip smoothing, publish original path
                ROS_WARN("Skipping smoothing, publishing original path");
            }
            
            // Publish path
            publishPath(path);
            ROS_INFO("Path published!");
        } else {
            ROS_WARN("Failed to find path!");
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
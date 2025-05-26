#ifndef HYBRID_ASTAR_PLANNER_H
#define HYBRID_ASTAR_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <vector>

// Forward declarations for hybrid_astar_searcher classes
namespace planning {
    class HybridAstar;
    class Smoother;
    typedef Eigen::Vector3d Vec3d;
}

// Forward declaration for Visualizer class
class Visualizer;

class HybridAStarPlanner {
public:
    HybridAStarPlanner();
    
private:
    // Callbacks
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void gridMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    // Path publication
    void publishPath(const std::vector<planning::Vec3d>& path);
    
    // 清除之前的路径和可视化
    void clearPreviousPath();
    
    // 边界检查和安全访问
    bool isWithinGridMapBounds(const grid_map::Position& position);
    bool safelySetElevation(const grid_map::Position& position, float value);
    
    // Node handle
    ros::NodeHandle nh_;
    
    // Subscribers
    ros::Subscriber map_sub_;
    ros::Subscriber grid_map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    
    // Publishers
    ros::Publisher path_pub_;
    ros::Publisher inflated_map_pub_;
    ros::Publisher rs_path_pub_;
    
    // TF
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // Map data
    nav_msgs::OccupancyGrid current_map_;
    grid_map::GridMap current_grid_map_;
    bool map_received_ = false;
    
    // Start and goal data
    geometry_msgs::Pose start_pose_;
    bool start_received_ = false;
    
    // 地图范围参数
    double map_x_min_ = -std::numeric_limits<double>::max();
    double map_x_max_ = std::numeric_limits<double>::max();
    double map_y_min_ = -std::numeric_limits<double>::max();
    double map_y_max_ = std::numeric_limits<double>::max();
    
    // Planner parameters
    double heuristic_weight_;
    double inflation_radius_;
    double map_resolution_;
    double vehicle_length_;
    double vehicle_width_;
    double min_turning_radius_;
    double steering_angle_;
    int steering_angle_discrete_num_;
    double segment_length_;
    int max_iterations_;
    bool visualize_vehicle_;
    int max_planning_attempts_;
    double planning_timeout_;
    
    // Smoother parameters
    double smoother_alpha_ = 0.1;
    double smoother_beta_ = 0.5;
    int smoother_max_iterations_ = 500;
    double smoother_weight_collision_ = 0.5;
    double smoother_weight_curvature_ = 0.2;
    double smoother_weight_smoothness_ = 0.3;
    
    // Planning components
    std::shared_ptr<planning::HybridAstar> hybrid_astar_searcher_;
    std::shared_ptr<planning::Smoother> smoother_;
    std::shared_ptr<Visualizer> visualizer_;
};

#endif // HYBRID_ASTAR_PLANNER_H 
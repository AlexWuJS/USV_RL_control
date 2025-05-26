#ifndef A_STAR_PLANNER_H
#define A_STAR_PLANNER_H

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

class AStarPlanner {
private:
    ros::NodeHandle nh_;
    ros::Subscriber map_sub_;
    ros::Subscriber start_sub_;
    ros::Subscriber goal_sub_;
    ros::Publisher path_pub_;
    ros::Publisher inflated_map_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    nav_msgs::OccupancyGrid current_map_;
    bool map_received_ = false;
    bool start_received_ = false;
    geometry_msgs::Pose start_pose_;
    
    // A星算法参数
    double heuristic_weight_;
    bool diagonal_movement_;
    int max_iterations_;
    double map_resolution_;
    double inflation_radius_;
    
    struct Node {
        int x, y;
        double g_cost;  // 从起点到当前节点的实际代价
        double h_cost;  // 从当前节点到终点的估计代价
        double f_cost;  // f = g + h
        Node* parent;
        
        Node(int x_, int y_) : x(x_), y(y_), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
        
        bool operator==(const Node& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    struct NodeHash {
        size_t operator()(const Node* node) const {
            return std::hash<int>()(node->x) ^ (std::hash<int>()(node->y) << 1);
        }
    };
    
    struct NodeEqual {
        bool operator()(const Node* a, const Node* b) const {
            return a->x == b->x && a->y == b->y;
        }
    };
    
    struct NodeCompare {
        bool operator()(const Node* a, const Node* b) const {
            return a->f_cost > b->f_cost;
        }
    };

public:
    AStarPlanner();
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    void startCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    
private:
    bool isValid(int x, int y);
    double calculateHeuristic(int x1, int y1, int x2, int y2);
    std::vector<Node*> findPath(int start_x, int start_y, int goal_x, int goal_y);
    std::vector<Node*> reconstructPath(Node* goal_node);
    void publishPath(const std::vector<Node*>& path);
};

#endif // A_STAR_PLANNER_H 
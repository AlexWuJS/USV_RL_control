#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <cmath>

class SimpleDemo {
public:
    SimpleDemo() {
        // 获取参数
        nh_.param("map_resolution", map_resolution_, 0.05);
        nh_.param("map_width", map_width_, 100);  // 栅格数
        nh_.param("map_height", map_height_, 100);  // 栅格数
        nh_.param("publish_rate", publish_rate_, 10.0);  // Hz
        
        // 创建发布者
        grid_map_pub_ = nh_.advertise<grid_map_msgs::GridMap>("/grid_map", 1);
        occupancy_grid_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/map", 1);
        
        // 初始化地图
        initializeMap();
        
        // 创建定时器
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &SimpleDemo::timerCallback, this);
        
        ROS_INFO("Simple Demo Node initialized with map size: %d x %d at resolution: %.3f m/cell", 
                 map_width_, map_height_, map_resolution_);
    }
    
    void initializeMap() {
        // 设置地图大小和分辨率
        map_.setGeometry(grid_map::Length(map_width_ * map_resolution_, map_height_ * map_resolution_), 
                         map_resolution_, grid_map::Position(0.0, 0.0));
        
        // 添加高程层（用于障碍物标记）
        map_.add("elevation", 0.0);
        
        // 创建四周边界墙
        addBoundaryWalls();
        
        // 添加一些随机障碍物
        addObstacles();
    }
    
    void addBoundaryWalls() {
        ROS_INFO("Adding boundary walls");
        double map_length_x = map_width_ * map_resolution_;
        double map_length_y = map_height_ * map_resolution_;
        double wall_thickness = 0.5;  // 墙的厚度（米）
        
        // 获取地图边界
        double x_min = -map_length_x / 2.0;
        double x_max = map_length_x / 2.0;
        double y_min = -map_length_y / 2.0;
        double y_max = map_length_y / 2.0;
        
        // 遍历所有栅格
        for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map_.getPosition(*it, position);
            
            // 标记边界墙
            if (position.x() < x_min + wall_thickness || position.x() > x_max - wall_thickness ||
                position.y() < y_min + wall_thickness || position.y() > y_max - wall_thickness) {
                map_.at("elevation", *it) = 1.0;  // 1.0表示障碍物
            }
        }
    }
    
    void addObstacles() {
        ROS_INFO("Adding obstacles");
        // 添加一些常见的障碍物形状
        
        // 1. 中心矩形障碍物
        addRectangleObstacle(10.0, 5.0, 0.0, 0.0, 0.0);
        
        // 2. 几个圆形障碍物
        addCircleObstacle(3.0, 8.0, 8.0);
        addCircleObstacle(2.5, -10.0, -7.0);
        addCircleObstacle(4.0, -5.0, 12.0);
        
        // 3. 添加一些随机小障碍物
        addRandomObstacles(15, 1.0, 2.0);
        
        // 4. 添加一个迷宫样式的障碍物
        addMazeObstacle(15.0, -15.0, 20.0, 15.0);
    }
    
    void addRectangleObstacle(double length, double width, double x, double y, double angle) {
        // 创建一个矩形障碍物
        for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map_.getPosition(*it, position);
            
            // 转换到矩形坐标系
            double dx = position.x() - x;
            double dy = position.y() - y;
            
            // 旋转
            double rotated_x = dx * std::cos(angle) - dy * std::sin(angle);
            double rotated_y = dx * std::sin(angle) + dy * std::cos(angle);
            
            // 检查是否在矩形内
            if (std::abs(rotated_x) < length / 2.0 && std::abs(rotated_y) < width / 2.0) {
                map_.at("elevation", *it) = 1.0;
            }
        }
    }
    
    void addCircleObstacle(double radius, double x, double y) {
        // 创建一个圆形障碍物
        for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
            grid_map::Position position;
            map_.getPosition(*it, position);
            
            // 计算到圆心的距离
            double dx = position.x() - x;
            double dy = position.y() - y;
            double distance = std::sqrt(dx * dx + dy * dy);
            
            // 检查是否在圆内
            if (distance < radius) {
                map_.at("elevation", *it) = 1.0;
            }
        }
    }
    
    void addRandomObstacles(int count, double min_size, double max_size) {
        // 添加随机障碍物
        double map_length_x = map_width_ * map_resolution_;
        double map_length_y = map_height_ * map_resolution_;
        
        // 获取地图边界（缩小一点避免障碍物超出边界）
        double x_min = -map_length_x / 2.0 + max_size;
        double x_max = map_length_x / 2.0 - max_size;
        double y_min = -map_length_y / 2.0 + max_size;
        double y_max = map_length_y / 2.0 - max_size;
        
        for (int i = 0; i < count; ++i) {
            // 随机位置
            double x = x_min + (x_max - x_min) * (double)rand() / RAND_MAX;
            double y = y_min + (y_max - y_min) * (double)rand() / RAND_MAX;
            
            // 随机大小
            double size = min_size + (max_size - min_size) * (double)rand() / RAND_MAX;
            
            // 50%概率创建圆形，50%概率创建矩形
            if (rand() % 2 == 0) {
                addCircleObstacle(size, x, y);
            } else {
                double angle = 2.0 * M_PI * (double)rand() / RAND_MAX;
                addRectangleObstacle(size * 2.0, size, x, y, angle);
            }
        }
    }
    
    void addMazeObstacle(double x, double y, double width, double height) {
        // 创建一个简单的迷宫样式障碍物
        double wall_thickness = 0.5;
        
        // 外墙
        addRectangleObstacle(width, wall_thickness, x, y - height / 2.0, 0.0);
        addRectangleObstacle(width, wall_thickness, x, y + height / 2.0, 0.0);
        addRectangleObstacle(wall_thickness, height, x - width / 2.0, y, 0.0);
        addRectangleObstacle(wall_thickness, height, x + width / 2.0, y, 0.0);
        
        // 内部隔断
        double section_width = width / 3.0;
        
        // 水平隔断
        addRectangleObstacle(2 * section_width, wall_thickness, x - section_width / 2.0, y - height / 4.0, 0.0);
        addRectangleObstacle(2 * section_width, wall_thickness, x + section_width / 2.0, y + height / 4.0, 0.0);
        
        // 垂直隔断
        addRectangleObstacle(wall_thickness, height / 2.0, x - section_width, y + height / 4.0, 0.0);
        addRectangleObstacle(wall_thickness, height / 2.0, x + section_width, y - height / 4.0, 0.0);
    }
    
    void timerCallback(const ros::TimerEvent& event) {
        // 添加一些动态更新（例如移动障碍物）
        
        // 发布GridMap
        publishGridMap();
        
        // 发布OccupancyGrid
        publishOccupancyGrid();
    }
    
    void publishGridMap() {
        // 添加时间戳
        map_.setTimestamp(ros::Time::now().toNSec());
        
        // 转换为消息
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(map_, message);
        
        // 发布
        grid_map_pub_.publish(message);
    }
    
    void publishOccupancyGrid() {
        // 转换为OccupancyGrid消息
        nav_msgs::OccupancyGrid grid;
        grid_map::GridMapRosConverter::toOccupancyGrid(map_, "elevation", 0.0, 1.0, grid);
        
        // 发布
        occupancy_grid_pub_.publish(grid);
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher grid_map_pub_;
    ros::Publisher occupancy_grid_pub_;
    ros::Timer timer_;
    
    grid_map::GridMap map_;
    double map_resolution_;
    int map_width_;
    int map_height_;
    double publish_rate_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_demo_node");
    SimpleDemo demo;
    ros::spin();
    return 0;
} 
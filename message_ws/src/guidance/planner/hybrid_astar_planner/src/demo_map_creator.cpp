#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include <random>

int main(int argc, char** argv) {
    ros::init(argc, argv, "demo_map_creator");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    // 发布器
    ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
    ros::Publisher grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("/grid_map", 1, true);
    
    // 参数
    double map_resolution;
    int map_width, map_height;
    double publish_rate;
    
    private_nh.param("map_resolution", map_resolution, 0.05);
    private_nh.param("map_width", map_width, 400);
    private_nh.param("map_height", map_height, 400);
    private_nh.param("publish_rate", publish_rate, 10.0);
    
    // 创建一个简单的演示地图（一个空旷区域加上一些随机障碍物和迷宫结构）
    nav_msgs::OccupancyGrid map;
    map.header.frame_id = "map";
    map.info.resolution = map_resolution;
    map.info.width = map_width;
    map.info.height = map_height;
    
    // 将地图中心点设置为更大的范围，这样可以覆盖更大的坐标区域
    double center_x = 100.0; // 地图中心x坐标
    double center_y = 0.0;   // 地图中心y坐标
    map.info.origin.position.x = center_x - (map_width * map_resolution / 2.0);
    map.info.origin.position.y = center_y - (map_height * map_resolution / 2.0);
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    
    map.data.resize(map_width * map_height, 0);
    
    ROS_INFO("创建演示地图: %dx%d, 分辨率: %.3f, 原点: (%.2f, %.2f)",
              map_width, map_height, map_resolution,
              map.info.origin.position.x, map.info.origin.position.y);
    
    // 随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(0.0, 1.0);
    
    // 创建边界墙
    int border_width = 5;
    for (int i = 0; i < map_width; ++i) {
        for (int j = 0; j < map_height; ++j) {
            if (i < border_width || i >= map_width - border_width ||
                j < border_width || j >= map_height - border_width) {
                map.data[j * map_width + i] = 100;
            }
        }
    }
    
    // 创建一些障碍物
    int num_obstacles = 15;
    for (int obs = 0; obs < num_obstacles; ++obs) {
        int obs_x = map_width / 4 + static_cast<int>(dis(gen) * map_width / 2);
        int obs_y = map_height / 4 + static_cast<int>(dis(gen) * map_height / 2);
        int obs_radius = 5 + static_cast<int>(dis(gen) * 15);
        
        for (int i = -obs_radius; i <= obs_radius; ++i) {
            for (int j = -obs_radius; j <= obs_radius; ++j) {
                if (i*i + j*j <= obs_radius * obs_radius) {
                    int x = obs_x + i;
                    int y = obs_y + j;
                    if (x >= 0 && x < map_width && y >= 0 && y < map_height) {
                        map.data[y * map_width + x] = 100;
                    }
                }
            }
        }
    }
    
    // 添加一个简单的迷宫结构
    int maze_start_x = map_width / 4;
    int maze_start_y = map_height / 4;
    int maze_width = map_width / 2;
    int maze_height = map_height / 2;
    int wall_thickness = 3;
    
    // 水平墙
    for (int i = 0; i < 3; ++i) {
        int y = maze_start_y + (i + 1) * maze_height / 4;
        for (int x = maze_start_x; x < maze_start_x + maze_width; ++x) {
            for (int t = -wall_thickness/2; t <= wall_thickness/2; ++t) {
                int yy = y + t;
                if (yy >= 0 && yy < map_height && x >= 0 && x < map_width) {
                    map.data[yy * map_width + x] = 100;
                }
            }
        }
        // 在每面墙上创建一个开口
        int opening_width = 10;
        int opening_x = maze_start_x + static_cast<int>(dis(gen) * (maze_width - opening_width));
        for (int x = opening_x; x < opening_x + opening_width; ++x) {
            for (int t = -wall_thickness/2; t <= wall_thickness/2; ++t) {
                int yy = y + t;
                if (yy >= 0 && yy < map_height && x >= 0 && x < map_width) {
                    map.data[yy * map_width + x] = 0;
                }
            }
        }
    }
    
    // 垂直墙
    for (int i = 0; i < 3; ++i) {
        int x = maze_start_x + (i + 1) * maze_width / 4;
        for (int y = maze_start_y; y < maze_start_y + maze_height; ++y) {
            for (int t = -wall_thickness/2; t <= wall_thickness/2; ++t) {
                int xx = x + t;
                if (xx >= 0 && xx < map_width && y >= 0 && y < map_height) {
                    map.data[y * map_width + xx] = 100;
                }
            }
        }
        // 在每面墙上创建一个开口
        int opening_height = 10;
        int opening_y = maze_start_y + static_cast<int>(dis(gen) * (maze_height - opening_height));
        for (int y = opening_y; y < opening_y + opening_height; ++y) {
            for (int t = -wall_thickness/2; t <= wall_thickness/2; ++t) {
                int xx = x + t;
                if (xx >= 0 && xx < map_width && y >= 0 && y < map_height) {
                    map.data[y * map_width + xx] = 0;
                }
            }
        }
    }
    
    // 创建GridMap
    grid_map::GridMap grid_map;
    grid_map.setFrameId("map");
    grid_map.setGeometry(grid_map::Length(map_width * map_resolution, map_height * map_resolution), 
                          map_resolution, grid_map::Position(center_x, center_y));
    grid_map.add("elevation", 0.0);
    
    // 填充障碍物
    grid_map::Matrix& data = grid_map["elevation"];
    for (grid_map::GridMapIterator it(grid_map); !it.isPastEnd(); ++it) {
        grid_map::Position position;
        grid_map.getPosition(*it, position);
        grid_map::Index index;
        grid_map.getIndex(position, index);
        
        // 将position转换为map索引
        int map_i = static_cast<int>((position.x() - map.info.origin.position.x) / map_resolution);
        int map_j = static_cast<int>((position.y() - map.info.origin.position.y) / map_resolution);
        
        if (map_i >= 0 && map_i < map_width && map_j >= 0 && map_j < map_height) {
            if (map.data[map_j * map_width + map_i] > 50) {
                data(index(0), index(1)) = 1.0;  // 障碍物
            } else {
                data(index(0), index(1)) = 0.0;  // 自由空间
            }
        }
    }
    
    ROS_INFO("演示地图创建完成");
    
    // 发布周期
    ros::Rate rate(publish_rate);
    while (ros::ok()) {
        // 更新时间戳
        map.header.stamp = ros::Time::now();
        grid_map.setTimestamp(ros::Time::now().toNSec());
        
        // 发布地图
        map_pub.publish(map);
        
        // 发布GridMap
        grid_map_msgs::GridMap message;
        grid_map::GridMapRosConverter::toMessage(grid_map, message);
        grid_map_pub.publish(message);
        
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
} 
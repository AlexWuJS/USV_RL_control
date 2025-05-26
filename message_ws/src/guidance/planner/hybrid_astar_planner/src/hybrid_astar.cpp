/**
 * @file hybrid_astar.cpp
 * @author jiaxier (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2023-10-08
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#include "hybrid_astar_searcher/hybrid_astar.h"


using namespace std;


namespace planning
{


HybridAstar::HybridAstar(grid_map::GridMap map) {
    map_ = map;
    xy_resolution_ = map.getResolution();
    theta_resolution_ = 0.1;
    
    // 扩大地图边界，原先是从起点减去25，现在改为减去50，给予更大的规划空间
    x_min_ = map.getStartIndex()(0) * xy_resolution_ - 50;
    x_max_ = x_min_ + map.getLength().x() + 50; // 在右侧也增加50的空间
    y_min_ = map.getStartIndex()(1) * xy_resolution_ - 50;
    y_max_ = y_min_ + map.getLength().y() + 50; // 在上侧也增加50的空间
    
    map_size_x_ = map.getSize()(0); 
    map_size_y_ = map.getSize()(1);
    
    cout << "map_size_x_" << map_size_x_ << endl;
    cout << "getStartIndex()(0)" << map.getStartIndex()(0) << endl;
    cout << "地图边界 x_min_: " << x_min_ << ", x_max_: " << x_max_ << endl;
    cout << "地图边界 y_min_: " << y_min_ << ", y_max_: " << y_max_ << endl;
    
    grid_search_ptr_ = make_unique<GridSearch>(map_);
    reed_shepp_generator_ =
      std::make_shared<ReedShepp>(0.3, 0.7);
    
}

HybridAstar::~HybridAstar() {}

/**
 * @brief 混合A星搜索
 * 
 * @param start_pose 
 * @param goal_pose 
 * @param result 
 * @return true 
 * @return false 
 */
bool HybridAstar::plan(Vec3d start_pose, Vec3d goal_pose, HybridAstarResult &result) {
    open_set_.clear();
    close_set_.clear();
    open_pq_ = decltype(open_pq_)();
    final_node_ = nullptr;
    auto hybrid_astar_start = chrono::high_resolution_clock::now();
    

    Vec3i start_pose_map = getIndexFromPose(start_pose);
    Vec3i goal_pose_map = getIndexFromPose(goal_pose);

    start_node_.reset(new Node3d({start_pose(0)}, {start_pose(1)}, {start_pose(2)}));
    goal_node_.reset(new Node3d({goal_pose(0)}, {goal_pose(1)}, {goal_pose(2)}));

    // 删除起点合法性检查，始终接受起点
    // if (!validityCheck(start_node_)) {
    //    cout << "起点不合理" << endl;
    //    return false;
    // }

    // 删除终点合法性检查，始终接受终点
    // if (!validityCheck(goal_node_)) {
    //    cout << "终点不合理" << endl;
    //    return false;
    // }

    //生成Heuristic 表
    auto dp_map_start = chrono::high_resolution_clock::now();
    grid_search_ptr_->generateDpMap({goal_pose(0), goal_pose(1)});
    auto dp_map_end = chrono::high_resolution_clock::now();
    chrono::duration<double> dp_map_use_time = dp_map_end - dp_map_start;
    


    start_node_->setIndex(start_pose_map, map_size_x_, map_size_y_);
    open_set_.emplace(start_node_->getIndex(), start_node_);
    open_pq_.emplace(start_node_->getIndex(), start_node_->getCost());

    int explored_node_num = 0;
    //vis.clear();
    while (!open_pq_.empty()) {
        //cout << "循环" << endl;
        int cur_index = open_pq_.top().first;
        cout << "cur_index" << cur_index << endl;
        open_pq_.pop();

        shared_ptr<Node3d> cur_node = open_set_[cur_index];
        
        //vis.publishExploredNodes({cur_node->getX(), cur_node->getY(), cur_node->getTheta()});

        //看看能否用RS曲线直接连接到终点
        if (AnalyticExpansion(cur_node)) {
            cout << "直接连接成功" << endl;
            break;
        }
        //cout << "没连接成功" << endl;

        close_set_.emplace(cur_index, cur_node);

        for (int i = 0; i < next_node_num_; ++i) {
            //cout << "expand " << i << "node" << endl;
            shared_ptr<Node3d> next_node = nextNodeGenerator(cur_node, i);
            //超出边界了
            if (next_node == nullptr) {
                continue;
            }            
            //cout << "node" << i << "is in map" << endl;

            //判断是否在closelist
            Vec3i index111 = getIndexFromPose({next_node->getX(), next_node->getY(), next_node->getTheta()});
            next_node->setIndex(index111, map_size_x_, map_size_y_);
            //cout << "node" << i << "index :" << next_node->getIndex() << endl;
            if (close_set_.find(next_node->getIndex()) != close_set_.end()) {
                //cout << "node " << i << "is in close set" << endl;
                continue;
            }
            

            if (!validityCheck(next_node)) {
                continue;
            }
            //cout << "node " << i << "is valid" << endl;

            if (open_set_.find(next_node->getIndex()) == open_set_.end()) {
                explored_node_num++;
                //cout << "探索的节点个数：" << explored_node_num << endl;
                
                calculateNodeCost(cur_node, next_node);
                
                open_pq_.emplace(next_node->getIndex(), next_node->getCost());
                open_set_.emplace(next_node->getIndex(), next_node);
            }
        }
    }

    if (final_node_ == nullptr) {
        cout << "搜索失败" << endl;
        return false;
    }

    if (!getHybridAstarResult(result)) {
        cout << "未得到解" << endl;
    }
    cout << "探索的节点个数：" << explored_node_num << endl;
    cout << "dp map use time:" << dp_map_use_time.count() * 1000 << "ms" << endl;
    auto hybrid_astar_end = chrono::high_resolution_clock::now();
    chrono::duration<double> hybrid_astar_use_time = hybrid_astar_end - hybrid_astar_start;
    cout << "hybrid astar use time:" << hybrid_astar_use_time.count() * 1000 << "ms" << endl;
    return true;

}

bool HybridAstar::getHybridAstarResult(HybridAstarResult &result) {
    shared_ptr<Node3d> cur_node = final_node_;
    vector<double> res_x;
    vector<double> res_y;
    vector<double> res_theta;

    while (cur_node->getParent() != nullptr) {
        vector<double> x = cur_node->getXs();
        vector<double> y = cur_node->getYs();
        vector<double> theta = cur_node->getThetas();
        reverse(x.begin(), x.end());
        reverse(y.begin(), y.end());
        reverse(theta.begin(), theta.end());

        x.pop_back();
        y.pop_back();
        theta.pop_back();

        res_x.insert(res_x.end(), x.begin(), x.end());
        res_y.insert(res_y.end(), y.begin(), y.end());
        res_theta.insert(res_theta.end(), theta.begin(), theta.end());
        cur_node = cur_node->getParent();
    }
    res_x.push_back(cur_node->getX());
    res_y.push_back(cur_node->getY());
    res_theta.push_back(cur_node->getTheta());
    reverse(res_x.begin(), res_x.end());
    reverse(res_y.begin(), res_y.end());
    reverse(res_theta.begin(), res_theta.end());


    result.x = res_x;
    result.y = res_y;
    result.theta = res_theta;
    //cout << "res size:" << res_x.size() << endl;
    //cout << "得到结果" << endl;
    return true;
}

shared_ptr<Node3d> HybridAstar::nextNodeGenerator(shared_ptr<Node3d> cur_node, int next_node_idx) {
    double steering = 0.0;
    double traveled_distance = 0.0;
    
    double delta_steering = 2 * max_steer_angle_ / (next_node_num_ / 2 - 1);
    //cout << "扩展节点" << endl;
    //cout << "next_node_idx :" << next_node_idx << endl;

    //得到正向反向移动距离和打角
    if (next_node_idx < next_node_num_ / 2) {
        steering = -max_steer_angle_ + delta_steering * next_node_idx;
        traveled_distance = step_size_;
    } else {
        steering = -max_steer_angle_ + delta_steering * (next_node_idx - next_node_num_ / 2);
        traveled_distance = -step_size_;
    }
    //cout << "steer:" << steering << endl;
    double steering1 = steering / 180 * M_PI; 
    //cout << "steer1:" << steering1 << endl;


    //存储节点的一堆中间点
    double arc = sqrt(2) * xy_resolution_; //可以设置想走多长，这个是栅格对角
    arc = 2.1; //可以设置想走多长
    //cout << "111111" << endl;
    vector<double> traversed_x;
    vector<double> traversed_y;
    vector<double> traversed_theta;
    double cur_x = cur_node->getX();
    double cur_y = cur_node->getY();
    double cur_theta = cur_node->getTheta();
    //cout << "11222" << endl;

    traversed_x.push_back(cur_x);  //把上一个节点也加进去了
    traversed_y.push_back(cur_y);
    traversed_theta.push_back(cur_theta);
    //cout << "arc / step_size_:" << arc / step_size_ << endl;
    //cout << "112223333" << endl;


    for (int i = 0; i < arc / step_size_; ++i) {
        double next_x = cur_x + traveled_distance * cos(cur_theta);
        double next_y = cur_y + traveled_distance * sin(cur_theta);
        double next_theta = cur_theta + traveled_distance * tan(steering1) / wheel_base_;
        //cout << "turn radius:" << 1 / (tan(steering1) / wheel_base_) << endl;
        mod2Pi(next_theta);
        

        traversed_x.push_back(next_x);
        traversed_y.push_back(next_y);
        traversed_theta.push_back(next_theta);

        cur_x = next_x;
        cur_y = next_y;
        cur_theta = next_theta;
    }
    //cout << "xuanhuanwan" << endl;
    //cout << traversed_x.size() << endl;

    //超出边界就返回nullptr
    if (traversed_x.back() < x_min_ || traversed_x.back() > x_max_ ||
        traversed_y.back() < y_min_ || traversed_y.back() > y_max_) {
        //cout << "超出边界" << endl;
        return nullptr;
    }
    

    shared_ptr<Node3d> next_node = make_shared<Node3d>(traversed_x, traversed_y, traversed_theta);
    next_node->setParent(cur_node);
    next_node->setDirection(traveled_distance > 0.0);
    next_node->setSteering(steering);
    //cout << "xyt:" << traversed_x.back() << " " << traversed_y.back() << " " << traversed_theta.back() << endl;
    return next_node;

}

double HybridAstar::TrajCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
    double piecewise_cost = 0.0;

    if (next_node->getDirection()) {
        piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_forward_penalty_;
    } else {
        piecewise_cost += (next_node->getStepSize() - 1) * step_size_ * traj_back_penalty_;
    }

    if (cur_node->getDirection() != next_node->getDirection()) {
        piecewise_cost += traj_gear_switch_penalty_;
    }

    piecewise_cost += traj_steer_change_penalty_ * std::abs(
        next_node->getSteering() - cur_node->getSteering()
    );

    piecewise_cost += traj_steer_penalty_ * std::abs(next_node->getSteering());

    return piecewise_cost;
    //各种惩罚待补充
}

double HybridAstar::calculateNodeCost(std::shared_ptr<Node3d> cur_node, std::shared_ptr<Node3d> next_node) {
    next_node->setTrajCost(cur_node->getTrajCost() + TrajCost(cur_node, next_node));
    //holo with obs
    double holo_heu_cost = 0.0;
    holo_heu_cost = grid_search_ptr_->lookupInDpMap({next_node->getX(), next_node->getY()});
    cout << "holo heu cost:" << holo_heu_cost << endl;

    //nonholo without obs
    auto non_holo_heu_start = chrono::system_clock::now();
    ob::StateSpacePtr space(std::make_shared<ob::ReedsSheppStateSpace>(3));
    ob::SE2StateSpace::StateType * rsStart = (ob::SE2StateSpace::StateType*)space->allocState();
    ob::SE2StateSpace::StateType* rsEnd = (ob::SE2StateSpace::StateType*)space->allocState();
    rsStart->setXY(next_node->getX(), next_node->getY());
    rsStart->setYaw(next_node->getTheta());
    rsEnd->setXY(goal_node_->getX(), goal_node_->getY());
    rsEnd->setYaw(goal_node_->getTheta());
    double RS_heu_cost = space->distance(rsStart, rsEnd);
    auto non_holo_heu_end = chrono::system_clock::now();
    chrono::duration<double> non_holo_use_time = non_holo_heu_end - non_holo_heu_start;
    cout << "nonholo use time:" << non_holo_use_time.count() * 1000 << "ms" << endl;

    cout << "RS heu cost: " << RS_heu_cost << endl;
    next_node->setHeuristicCost(max(RS_heu_cost, holo_heu_cost));
}

vector<Vec2d> HybridAstar::calculateCarBoundingBox(Vec3d pose) {
    vector<Vec2d> vertices;
    double shift_distance = length_ / 2 - back_edge_to_center_;
    Vec2d center = {pose(0) + shift_distance * std::cos(pose(2)),
                    pose(1) + shift_distance * std::sin(pose(2))};
    const double dx1 = std::cos(pose(2)) * length_ / 2;
    const double dy1 = std::sin(pose(2)) * length_ / 2;
    const double dx2 = std::sin(pose(2)) * width_ / 2;
    const double dy2 = -std::cos(pose(2)) * width_ / 2;
    vertices.emplace_back(center(0) + dx1 + dx2, center(1) + dy1 + dy2);
    vertices.emplace_back(center(0) + dx1 - dx2, center(1) + dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 - dx2, center(1) - dy1 - dy2);
    vertices.emplace_back(center(0) - dx1 + dx2, center(1) - dy1 + dy2); //顺时针
    return vertices;

}
/**
 * @brief 查询一条线上的点是否在障碍物里
 * 
 * @param x0 
 * @param y0 
 * @param x1 
 * @param y1 
 * @return true 
 * @return false 
 */
bool HybridAstar::isLinecollision(double x0, double y0, double x1, double y1) {
    // 使用更安全的点数计算方法，确保点的密度足够
    double distance = std::sqrt((x1-x0)*(x1-x0) + (y1-y0)*(y1-y0));
    int check_point_num = std::max(static_cast<int>(distance / (xy_resolution_ * 0.5)) + 1, 2);

    double delta_x = (x1 - x0) / (check_point_num - 1);
    double delta_y = (y1 - y0) / (check_point_num - 1);

    double cur_x = x0;
    double cur_y = y0;
    
    // 检查线段上的所有点
    for (int i = 0; i < check_point_num; ++i) {
        // 跳过地图外的点，这些点在validityCheck中已经处理
        if (!isInMap(cur_x, cur_y)) {
            // 已修改为总是返回true，所以这里不会执行
            cur_x += delta_x;
            cur_y += delta_y;
            continue;
        }

        try {
            // 安全地查询地图数据
            grid_map::Position position(cur_x, cur_y);
            
            // 检查点是否在地图范围内
            if (map_.isInside(position)) {
                // 检查点是否是障碍物
                if (map_.atPosition("elevation", position) > 0) {
                    // 是障碍物，检测到碰撞
                    return true;
                }
            }
        } catch (const std::exception& e) {
            // 如果发生异常，记录日志并继续检查下一个点
            std::cout << "地图访问异常在 (" << cur_x << ", " << cur_y << "): " << e.what() << std::endl;
        }

        cur_x += delta_x;
        cur_y += delta_y;
    }
    // 没有检测到碰撞
    return false;
}

/**
 * @brief 验证节点是否可行，需要得到车的轮廓，然后用上边的判断四条边是否穿过障碍物
 * 
 * @param node 
 * @return true 
 * @return false 
 */
bool HybridAstar::validityCheck(std::shared_ptr<Node3d> node) {
    // 如果是起点或终点节点，直接返回true，表示合法
    if (node == start_node_ || node == goal_node_) {
        return true;
    }
    
    int node_step_size = node->getStepSize();
    const auto traversed_x = node->getXs();
    const auto traversed_y = node->getYs();
    const auto traversed_theta = node->getThetas();

    int start_index = 0;
    if (node_step_size > 1) {  //不止一个节点，有中间节点
        start_index = 1;   //第一个是上一个节点的x y theta
    }

    //遍历节点集合里的点 (对于非起点终点的节点，仍然进行碰撞检测)
    for (int i = start_index; i < node_step_size; ++i) {
        try {
            // 计算车辆边界框
            auto vertices = calculateCarBoundingBox({traversed_x[i], traversed_y[i], traversed_theta[i]});
            
            // 只检查在地图内的点的碰撞
            for (int j = 0; j < vertices.size(); ++j) {
                // 如果车辆某个顶点超出地图范围，跳过该顶点的碰撞检测
                if (vertices[j].x() < x_min_ || vertices[j].x() > x_max_ ||
                    vertices[j].y() < y_min_ || vertices[j].y() > y_max_) {
                    continue;
                }
                
                // 检测车辆边缘线段是否与障碍物碰撞
                if (isLinecollision(vertices[j].x(), vertices[j].y(), 
                                vertices[(j+1) % 4].x(), vertices[(j+1) % 4].y())) {
                    // 发生碰撞，节点无效
                    return false;                    
                }
            }
        } catch (const std::exception& e) {
            // 捕获任何异常，记录并返回无效
            std::cout << "车辆碰撞检测异常: " << e.what() << std::endl;
            return false;
        }
    }
    // 所有检查通过，节点有效
    return true;
}
Vec3i HybridAstar::getIndexFromPose(Vec3d pose) {
    Vec3i index;
    index(0) = static_cast<int>((pose(0) - x_min_) / xy_resolution_);
    index(1) = static_cast<int>((pose(1) - y_min_) / xy_resolution_);
    mod2Pi(pose(2));
    index(2) = static_cast<int>((pose(2)) / theta_resolution_);
    return index;
}

/**
 * @brief 直接用RS曲线连接到终点
 * 
 * @param current_node 
 * @return true 
 * @return false 
 */
bool HybridAstar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
    std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
        std::make_shared<ReedSheppPath>();
    if (!reed_shepp_generator_->ShortestRSP(current_node, goal_node_,
                                            *reeds_shepp_to_check)) {
        return false;
    }
    // std::cout << "ShortestRSP found";

    std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(new Node3d(
      reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
    
    if (!validityCheck(node)) {
        return false;
    }

    std::shared_ptr<Node3d> goal_node = std::shared_ptr<Node3d>(new Node3d(
    reeds_shepp_to_check->x, reeds_shepp_to_check->y, reeds_shepp_to_check->phi));
    goal_node->setParent(current_node);
    close_set_.emplace(goal_node->getIndex(), goal_node);
    final_node_ = goal_node;
    return true;
}


void HybridAstar::mod2Pi(double &angle) {
    if (angle >  M_PI) {
        angle -= 2 *M_PI;
    } 

    if (angle < - M_PI) {
        angle += 2 * M_PI;
    }
}

bool HybridAstar::isInMap(double x, double y) {
    // 取消边界限制，永远返回true表示点在地图内
    return true;
}

    
} // namespace planning


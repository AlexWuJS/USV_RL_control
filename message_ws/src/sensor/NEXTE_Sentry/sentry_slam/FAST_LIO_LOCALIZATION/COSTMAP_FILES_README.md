# 代价地图相关文件说明

本目录包含了所有与代价地图（Costmap）相关的配置和启动文件。

## 📁 文件结构

### 参数配置文件 (param/)
- **costmap_common_params.yaml** - 全局和局部代价地图的通用参数
  - 机器人几何参数（robot_radius, footprint）
  - 障碍物检测范围配置
  - VoxelLayer 3D点云障碍物检测配置
  - 膨胀层（inflation layer）配置
  
- **global_costmap_params.yaml** - 全局代价地图专用参数
  - 使用静态地图（map坐标系）
  - 更新频率和发布频率
  - 静态层配置
  
- **local_costmap_params.yaml** - 局部代价地图专用参数
  - 滚动窗口模式（跟随机器人移动）
  - 使用camera_init坐标系
  - 局部地图尺寸和分辨率
  
- **rosconsole_costmap.config** - 代价地图日志输出配置（可选）

### 启动文件 (launch/)
- **costmap_nodes.launch** - 独立启动代价地图节点
  - 启动全局costmap节点
  - 启动局部costmap节点
  - 加载所有相关参数文件

## 🔧 使用方法

### 方法1: 独立启动代价地图
```bash
roslaunch fast_lio_localization costmap_nodes.launch
```

### 方法2: 作为其他launch文件的一部分
在其他launch文件中引用这些参数文件：
```xml
<rosparam file="$(find fast_lio_localization)/param/global_costmap_params.yaml" command="load" />
<rosparam file="$(find fast_lio_localization)/param/costmap_common_params.yaml" command="load" />
<rosparam file="$(find fast_lio_localization)/param/local_costmap_params.yaml" command="load" />
```

## 📝 注意事项

1. **坐标系配置**
   - 全局costmap使用 `map` 坐标系（历史3D地图）
   - 局部costmap使用 `camera_init` 坐标系（SLAM局部参考系）
   - 机器人基座使用 `body` 坐标系

2. **点云话题**
   - 使用 `/cloud_registered_body` 作为3D点云输入
   - 传感器坐标系为 `body`

3. **相关launch文件**
   - `sentry_build_map.launch` - 建图时使用代价地图
   - `sentry_movebase.launch` - move_base导航时使用代价地图

## 🔗 依赖关系

- 需要 `costmap_2d` ROS包
- 需要FAST-LIO提供的点云数据
- 需要TF变换正确配置

## 📊 性能参数

- 全局costmap更新频率: 5.0 Hz
- 局部costmap更新频率: 5.0 Hz
- 局部地图尺寸: 20m × 20m
- 分辨率: 0.1 m/cell
- 障碍物检测范围: 15m
- 最大障碍物高度: 2.5m

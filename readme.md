## data_preprocessing模块：

### Subscribe:
- 原始数据点云 /kitti/velo/pointcloud
- 原始IMU数据  /kitti/oxts/imu
- 原始GNSS数据 /kitti/oxts/gps/fix
- velocity原始数据 /kitti/oxts/gps/vel
- IMU与Lidar之间的外参数

### Publish
- 时间同步并且经过畸变处理的点云 /synced_cloud
- 时间同步的GNSS数据(经纬度和东北天下的位置) /synced_gnss

## front_end模块：

### Subscribe:
- 时间同步之后的点云 /synced_cloud

### Publish:
- 激光里程计的位姿 /laser_odom

## back_end模块

### Subscribe:
- 预处理输出的点云 /synced_cloud (只用来保存)
- 预处理输出的GNSS位姿 /synced_gnss
- 雷达里程计的位姿 /laser_odom

### Publish:
- 当前雷达里程计的位姿转换到与GNSS同一坐标下的位姿 /transformed_odom
- 最新的关键帧 /key_frame
- 优化后的多个关键帧(nav/Path) /optimized_key_frames

## viewer模块

### Subscribe:
- 预处理输出的点云 /synced_cloud
- 后端的关键帧 /key_frame
- 后端当当前雷达里程计的位姿转换到与GNSS同一坐标下的位姿 /transformed_odom
- 后端已经优化的所有关键帧 /optimized_key_frames

### Publish:
- 发布当前帧的位姿（投影到已经优化的轨迹上去）/optimized_odom
- 当前帧点云 /current_scan
- 局部地图 /local_map
- 全局题图 /global_map

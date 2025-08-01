include "map_builder.lua"  
include "trajectory_builder.lua"  

options = {
  map_builder = MAP_BUILDER,                -- map_builder.lua的配置信息
  trajectory_builder = TRAJECTORY_BUILDER,  -- trajectory_builder.lua的配置信息
  
  map_frame = "map",                        -- 地图坐标系的名字
  tracking_frame = "base_footprint",        -- 将所有传感器数据转换到这个坐标系下
  published_frame = "odom",                 -- tf: map -> odom
  odom_frame = "odom",                      -- 里程计的坐标系名字
  provide_odom_frame = false,               -- 是否提供odom的tf, 如果为true,则tf树为map->odom->footprint
                                            -- 如果为false tf树为map->footprint
  publish_frame_projected_to_2d = false,    -- 是否将坐标系投影到平面上

  use_odometry = true,                     -- 是否使用里程计,如果使用要求一定要有odom的tf
  use_nav_sat = false,                      -- 是否使用gps
  use_landmarks = false,                    -- 是否使用landmark
  num_laser_scans = 1,                      -- 是否使用单线激光数据
  num_multi_echo_laser_scans = 0,           -- 是否使用multi_echo_laser_scans数据
  num_subdivisions_per_laser_scan = 1,      -- 1帧数据被分成几次处理,一般为1
  num_point_clouds = 0,                     -- 是否使用点云数据
  
  lookup_transform_timeout_sec = 0.2,       -- 查找tf时的超时时间
  submap_publish_period_sec = 0.3,          -- 发布数据的时间间隔
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,

  rangefinder_sampling_ratio = 1.,          -- 传感器数据的采样频率
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- Refrence:
-- https://zhuanlan.zhihu.com/p/675717750 (1)
-- https://www.waveshare.com/wiki/Cartographer_Map_Building (2)

-- Ref:1
MAP_BUILDER.use_trajectory_builder_2d = true  -- whether use 2D SLAM
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 35  -- Number of range data for submaps in the 2D track builder  
TRAJECTORY_BUILDER_2D.min_range = 0.2  -- ignore anything smaller than the robot radius, limiting it to the minimum scan range of the lidar
-- Our lidar provide noisy signal out of 5.0 M
TRAJECTORY_BUILDER_2D.max_range = 5.0  -- the maximum scanning range of the lidar
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0  -- Restricted to maximum LiDAR scanning range  
TRAJECTORY_BUILDER_2D.use_imu_data = false  -- whether use IMU data
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true  -- Whether to scan for matches using real-time loopback detection

-- Ref: 2
-- TRAJECTORY_BUILDER_2D.min_z = 0.2 -- / -0.8
-- TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.02
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 1.
-- Tuning resolution here and the launch file's parameters to ensure the obstacle darkness of the map
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.08
-- POSE_GRAPH.optimize_every_n_nodes = 70. -- 2 倍的 num_range_data 以上
-- POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
-- POSE_GRAPH.constraint_builder.max_constraint_distance = 15.

-- Ref: 1
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.1)  -- Modify 1.0 to 0.1, increased sensitivity to movement
POSE_GRAPH.constraint_builder.min_score = 0.65  -- Modify 0.55 to 0.65, the minium score of Fast csm, can be optimized above this score 
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7  -- Modify 0.6 as 0.7, Minimum global positioning score below which global positioning is considered currently inaccurate

-- The scan dot will be mark or erase:
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.65    -- default: 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.49   -- default: 0.49

return options
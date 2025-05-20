-- my_custom_robot_2d.lua (Example Snippet)

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",  -- Or your robot's main body frame, or imu_link if using IMU consistently
  published_frame = "base_link", -- The frame Cartographer will publish its pose estimate for
  odom_frame = "odom",
  provide_odom_frame = true,    -- Cartographer provides map->odom transform
  publish_frame_projected_to_2d = true, -- For 2D SLAM
  use_odometry = false,          -- Set to true if your Gazebo robot publishes odometry
  use_nav_sat = false,
  use_landmarks = false,         -- This being false is fine
  num_laser_scans = 1,          -- If you have one 2D LiDAR
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5.,
  trajectory_publish_period_sec = 30.,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1., -- <<< THIS LINE NEEDS TO BE PRESENT
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.min_range = 0.1         -- Adjust to your LiDAR's min range
TRAJECTORY_BUILDER_2D.max_range = 30.0        -- Adjust to your LiDAR's max range
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5. -- Use a value slightly larger than max_range
TRAJECTORY_BUILDER_2D.use_imu_data = false    -- Set to true if you have a reliable IMU in Gazebo and want to use it
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- ... other TRAJECTORY_BUILDER_2D parameters ...

-- If use_odometry = true, odometry settings might be within TRAJECTORY_BUILDER
-- Ensure Cartographer can find your odometry data via TF (odom_frame)

-- If use_imu_data = true, configure IMU settings:
-- TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 10.
-- The IMU frame ID is usually picked up via TF, ensure it's linked to your tracking_frame

-- POSE_GRAPH options are for loop closure and optimization
-- ...

return options
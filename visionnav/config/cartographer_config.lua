include "map_builder.lua"
include "trajectory_builder.lua"

-- Chest-worn RPLIDAR C1, no wheel odometry, no IMU.
-- Cartographer does scan-to-submap matching on its own, so it is the right backend for a
-- wearable: it publishes map -> odom and the static odom -> base_footprint does the rest.
options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
-- Body returns are already removed by scan_body_filter.py; this is a second guard.
TRAJECTORY_BUILDER_2D.min_range = 0.45
-- A chest-worn scan plane pitches with every step; beyond ~8 m it starts hitting floor/ceiling.
TRAJECTORY_BUILDER_2D.max_range = 8.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.0

-- No odometry prior: let the correlative matcher find the pose. Walking is ~1.4 m/s
-- (0.14 m per 10 Hz scan) and people turn their torso fast.
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.25
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(35.)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 10.
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 1e-1
-- Trust the scan more than the (non-existent) motion prior.
-- Pin the scan to the walls; a light translation prior (no odometry exists to trust).
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 20.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 40.

TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
-- Fewer, better-spaced nodes: less CPU and less jitter in the pose graph.
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.10
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(2.)

-- Clear dynamic obstacles faster: a miss outweighs a hit (defaults 0.55 / 0.49). Finished submaps are
-- frozen, so old ghosts only fade where newer submaps overlap them. Below ~0.44 thin walls start to erode.
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.insert_free_space = true

-- Smaller submaps drift less between loop closures in small indoor rooms.
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60

-- 0.05 m matches the RPLIDAR C1's ~3 cm range noise; 0.03 m would cost ~2.8x the cells for no real gain.
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.05
POSE_GRAPH.optimize_every_n_nodes = 20
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3
POSE_GRAPH.constraint_builder.max_constraint_distance = 10.
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.optimization_problem.huber_scale = 1e1

return options

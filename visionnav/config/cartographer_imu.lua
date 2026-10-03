-- cartographer_config.lua plus the chest MPU-6050 (mpu6050_imu.py on the Pi, /imu/data, 100 Hz), used by
-- laptop_brain.launch.py imu:=true (the assistant sets it when the Pi publishes /imu/data). The gyro predicts each
-- scan's rotation (fast torso turns) and gravity levels the scan when the chest leans. Cartographer must track the
-- IMU's own frame; poses are still published projected to 2-D (publish_frame_projected_to_2d).
include "cartographer_config.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- Which way is down: the accelerometer averaged over 2 s (default 10 s), since the chest's tilt changes whenever
-- the wearer leans; steps (~0.5 s) still average out.
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 2.

return options

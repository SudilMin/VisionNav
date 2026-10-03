-- cartographer_config.lua plus the chest MPU-6050 (mpu6050_imu.py on the Pi, /imu/data, 100 Hz).
-- laptop_brain.launch.py uses it with imu:=true (the assistant sets it when the Pi publishes /imu/data).
-- The gyro predicts each scan's rotation (a fast torso turn no longer has to be found by the scan matcher alone)
-- and gravity levels the scan when the chest leans, so leaning points are placed where they really are.
-- Cartographer moves the IMU's accelerations to no other point: it must track the IMU's own frame. Poses are
-- still published projected to 2-D (publish_frame_projected_to_2d).
include "cartographer_config.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- Which way is down: the accelerometer averaged over 2 s (default 10 s). Worn on a chest, the tilt changes whenever
-- the wearer leans; a 10 s average kept levelling the scans with an old tilt. Steps (~0.5 s) still average out.
-- Replays of a recorded walk (2026-10-01, 2 runs each): with this and the measured mount (sensor_tf.launch.py), the
-- live position jumped 0.9 m in all (without the IMU 0.8 m; with the old 10 s and 6 deg mount 2.6 m), and the map's
-- loop-closure corrections were 0.22-0.24 m at the 90th percentile (0.18-0.28 without the IMU, 0.27-0.28 before).
TRAJECTORY_BUILDER_2D.imu_gravity_time_constant = 2.

return options

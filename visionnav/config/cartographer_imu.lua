-- cartographer_config.lua plus the chest MPU-6050 (mpu6050_imu.py on the Pi, /imu/data, 100 Hz).
-- laptop_brain.launch.py uses it with imu:=true (the assistant sets it when the Pi publishes /imu/data).
-- The gyro predicts each scan's rotation (a fast torso turn no longer has to be found by the scan matcher alone)
-- and gravity levels the scan when the chest leans, so leaning points are placed where they really are.
-- Cartographer moves the IMU's accelerations to no other point: it must track the IMU's own frame. Poses are
-- still published projected to 2-D (publish_frame_projected_to_2d).
include "cartographer_config.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true

return options

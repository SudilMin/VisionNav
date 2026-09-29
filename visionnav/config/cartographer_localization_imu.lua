-- cartographer_localization.lua plus the chest MPU-6050 (see cartographer_imu.lua). A map saved with or
-- without the IMU can be loaded either way.
include "cartographer_localization.lua"

options.tracking_frame = "imu_link"
TRAJECTORY_BUILDER_2D.use_imu_data = true

return options

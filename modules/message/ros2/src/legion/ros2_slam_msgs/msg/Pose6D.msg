# Preintegrated IMU/LiDAR state at the time of IMU measurements
# Offset time relative to the base time (seconds)
float64 offset_time

# Preintegrated total acceleration in global/world frame
float64[3] acc

# Unbiased angular velocity in body frame
float64[3] gyr

# Preintegrated velocity in global/world frame
float64[3] vel

# Preintegrated position in global/world frame
float64[3] pos

# Preintegrated rotation matrix (row-major 9 elements) in global/world frame
float64[9] rot

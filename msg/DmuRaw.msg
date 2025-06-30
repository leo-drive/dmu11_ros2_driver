std_msgs/Header header

# Number of message
uint16 msg_count

# Gyro rates
geometry_msgs/Vector3 angular_rate
float64[9] angular_rate_covariance # Row major about x, y, z axes

# Linear acceleration
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # Row major x, y z

# Average Imu temperature
float64 average_imu_temp

# Delta theta
geometry_msgs/Vector3 delta_theta
float64[9] delta_theta_covariance # Row major x, y z

# Delta velocity
geometry_msgs/Vector3 delta_velocity
float64[9] delta_velocity_covariance # Row major x, y z

# System flags
uint16 system_startup_flags
uint16 system_operat_flags
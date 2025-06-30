# DMU11 ROS 2 IMU Driver

This package provides a ROS 2-based driver for the DMU11 IMU sensor. It reads data via UART, parses the incoming packet, and publishes it as ROS2 message.

## Features

- UART-based communication with the IMU
- Packet synchronization and checksum validation
- Compatible with standard `sensor_msgs/msg/Imu` message type
- Orientation estimation via incremental delta angles
- Publishes raw IMU values using custom message format

## Supported ROS 2 Distributions

- ROS 2 Humble

## Installation

```bash
cd ~/ros2_ws/src
git clone git@github.com:leo-drive/dmu11_ros2_driver.git dmu11_ros2_driver
cd ..
colcon build --symlink-install --packages-select dmu11_ros2_driver
source install/setup.bash
```

## Launching the IMU

```bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch dmu11_ros2_driver dmu11.launch.xml
```

## Parameters

| Parameter Name        | Type    | Default       | Required on startup | Information                                                      |
|-----------------------|---------|---------------|----------------------|------------------------------------------------------------------|
| working_frequency     | Integer | 500           |                      | Frequency (Hz) at which the driver reads and publishes data.     |
| serial_config.port    | String  | /dev/ttyUSB0  |                      | Serial port to which the DMU11 IMU is connected.                 |
| frame_config.imu_frame| String  | imu_link      |                      | Frame ID used in the published messages.                         |


## Published Topics

| Topic Name | Message type | Description |
|----------|:----------:|:----------|
| /dmu11/Imu | sensor_msgs::msg::Imu  | Linear acceleration and angular velocity |
| /dmu11/pose | geometry_msgs::msg::PoseStamped  | Orientation represented as pose |
| /dmu11/dmu_raw | dmu11_ros2_driver::msg::DmuRaw> | Parsed raw data from DMU11 |

## Architecture

- SerialPort: Handles low-level UART communication with the IMU.
- Dmu11Parser: Parses binary data and converts it into usable ROS messages.
- Dmu11Receiver: Main ROS 2 node that initializes communication, processes frames, and publishes topics.

## DmuRaw.msg

```bash
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
```

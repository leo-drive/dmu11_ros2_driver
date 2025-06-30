#include "dmu11_ros2_driver/dmu11.hpp"

#include <chrono>
#include <memory>

namespace DMU11
{

    Dmu11Receiver::Dmu11Receiver(const rclcpp::NodeOptions &options)
        : rclcpp::Node("dmu11_receiver", options)
    {
        working_frequency_ = this->declare_parameter("working_frequency", 500);
        serial_port_ = this->declare_parameter("serial_config.port", "/dev/ttyUSB0");
        frame_id_ = this->declare_parameter("frame_config.imu_frame", "imu");

        RCLCPP_INFO(this->get_logger(), "Working Frequency     : %d Hz", working_frequency_);
        RCLCPP_INFO(this->get_logger(), "Serial Port           : %s", serial_port_.c_str());
        RCLCPP_INFO(this->get_logger(), "IMU Frame ID          : %s", frame_id_.c_str());

        try
        {
            serial_port_ptr_ = std::make_shared<SerialPort>(serial_port_.c_str());
            serial_port_ptr_->open();
            serial_port_ptr_->configure(460800U, 8, 'N', 1);
        }
        catch (const SerialPortException &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
            rclcpp::shutdown();
        }

        dmu11_parser_ptr_ = std::make_shared<Dmu11Parser>(std::bind(&Dmu11Receiver::new_frame_received, this), frame_id_);
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("dmu11/Imu", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(1000 / working_frequency_), std::bind(&Dmu11Receiver::timer_callback, this));
        dmu11_raw_pub_ = this->create_publisher<dmu11_ros2_driver::msg::DmuRaw>("dmu11/dmu_raw", 100);
        imu_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("dmu11/pose", 10);
        RCLCPP_INFO(this->get_logger(), "DMU11 Receiver node initialized.");
    }

    void Dmu11Receiver::timer_callback()
    {
        uint8_t buffer[512];
        try
        {
            int len = serial_port_ptr_->read(reinterpret_cast<char *>(buffer), sizeof(buffer));
            if (len > 0)
            {
                dmu11_parser_ptr_->parse_data(buffer, len);
            }
        }
        catch (const SerialPortException &e)
        {
            RCLCPP_ERROR(get_logger(), e.what());
        }
    }

    void Dmu11Receiver::new_frame_received()
    {
        dmu11_raw_pub_->publish(dmu11_parser_ptr_->get_dmu_raw_data());
        imu_pub_->publish(dmu11_parser_ptr_->get_imu_data());
        sensor_msgs::msg::Imu imu_orientation = dmu11_parser_ptr_->get_imu_data();
        geometry_msgs::msg::PoseStamped pose_stamp_msg;
        geometry_msgs::msg::Pose pose_msg;

        pose_stamp_msg.header.stamp = imu_orientation.header.stamp;
        pose_stamp_msg.header.frame_id = imu_orientation.header.frame_id;
        pose_msg.position.x = 0.0;
        pose_msg.position.y = 0.0;
        pose_msg.position.z = 0.0;
        pose_msg.orientation.x = imu_orientation.orientation.x;
        pose_msg.orientation.y = imu_orientation.orientation.y;
        pose_msg.orientation.z = imu_orientation.orientation.z;
        pose_msg.orientation.w = imu_orientation.orientation.w;
        pose_stamp_msg.pose = pose_msg;
        imu_pose_pub_->publish(pose_stamp_msg);        
    }

} // namespace DMU11
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(DMU11::Dmu11Receiver)
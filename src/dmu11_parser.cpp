#include "dmu11_ros2_driver/dmu11_parser.hpp"

#include <sstream>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>

namespace DMU11
{
    constexpr double gravity_constant = 9.80665; // m/s^2

    Dmu11Parser::Dmu11Parser(std::function<void()> callback, const std::string &frame_id)
        : frame_id_(frame_id), callback_(callback)
    {
    }
    void Dmu11Parser::set_callback(std::function<void()> callback)
    {
        callback_ = callback;
    }
    void Dmu11Parser::set_frame_id(const std::string &frame_id)
    {
        frame_id_ = frame_id;
    }

    float Dmu11Parser::to_float_be(const uint8_t *data, uint16_t *index)
    {
        uint32_t temp = (static_cast<uint32_t>(data[0]) << 24) |
                        (static_cast<uint32_t>(data[1]) << 16) |
                        (static_cast<uint32_t>(data[2]) << 8) |
                        (static_cast<uint32_t>(data[3]));

        float result;
        std::memcpy(&result, &temp, sizeof(float));
        if (index)
            *index += 4; 
        return result;
    }

    uint16_t Dmu11Parser::to_uint16_be(const uint8_t *data, uint16_t *index)
    {
        uint32_t temp = (static_cast<uint32_t>(data[0]) << 8) |
                        (static_cast<uint32_t>(data[1]));

        uint16_t result;
        std::memcpy(&result, &temp, sizeof(uint16_t));
        if (index)
            *index += 2; 
        return result;
    }

    int16_t Dmu11Parser::calculate_checksum(const uint8_t *packet)
    {
        uint32_t sum = 0;
        for (size_t i = 0; i < 66; i += 2)
        {
            uint16_t word = (static_cast<uint16_t>(packet[i]) << 8) | packet[i + 1];
            sum += word;
        }

        sum &= 0xFFFF;
        uint16_t checksum = (~sum + 1) & 0xFFFF;
        return static_cast<int16_t>(checksum); 
    }

    void Dmu11Parser::handle_packet(const uint8_t* packet)
    {
        uint16_t index = 2;
        const uint8_t *ptr = packet;

        raw_dmu_data_.msg_count = to_uint16_be(ptr + index, &index);
        raw_dmu_data_.angular_rate.x = to_float_be(ptr + index, &index);
        raw_dmu_data_.linear_acceleration.x = to_float_be(ptr + index, &index);
        raw_dmu_data_.linear_acceleration.y = to_float_be(ptr + index, &index);
        raw_dmu_data_.angular_rate.y = to_float_be(ptr + index, &index);
        raw_dmu_data_.angular_rate.z = to_float_be(ptr + index, &index);
        raw_dmu_data_.linear_acceleration.z = to_float_be(ptr + index, &index);

        imu_raw_.linear_acceleration.x = raw_dmu_data_.linear_acceleration.x * gravity_constant;
        imu_raw_.linear_acceleration.y = raw_dmu_data_.linear_acceleration.y * gravity_constant;
        imu_raw_.linear_acceleration.z = raw_dmu_data_.linear_acceleration.z * gravity_constant;

        imu_raw_.angular_velocity.x = raw_dmu_data_.angular_rate.x * M_PI / 180;
        imu_raw_.angular_velocity.y = raw_dmu_data_.angular_rate.y * M_PI / 180;
        imu_raw_.angular_velocity.z = raw_dmu_data_.angular_rate.z * M_PI / 180;

        index += 4; // Reserved
        raw_dmu_data_.average_imu_temp = to_float_be(ptr + index, &index);

        raw_dmu_data_.delta_theta.x = to_float_be(ptr + index, &index);
        raw_dmu_data_.delta_velocity.x = to_float_be(ptr + index, &index);
        raw_dmu_data_.delta_theta.y = to_float_be(ptr + index, &index);
        raw_dmu_data_.delta_velocity.y = to_float_be(ptr + index, &index);
        raw_dmu_data_.delta_theta.z = to_float_be(ptr + index, &index);
        raw_dmu_data_.delta_velocity.z = to_float_be(ptr + index, &index);

        raw_dmu_data_.system_startup_flags = to_uint16_be(ptr + index, &index);
        raw_dmu_data_.system_operat_flags = to_uint16_be(ptr + index, &index);

        roll_ += raw_dmu_data_.delta_theta.x * M_PI / 180;
        pitch_ += raw_dmu_data_.delta_theta.y * M_PI / 180;
        yaw_ += raw_dmu_data_.delta_theta.z * M_PI / 180;

        tf2::Quaternion tf_quat;
        tf_quat.setRPY(roll_, pitch_, yaw_);

        imu_raw_.orientation.x = tf_quat.x();
        imu_raw_.orientation.y = tf_quat.y();
        imu_raw_.orientation.z = tf_quat.z();
        imu_raw_.orientation.w = tf_quat.w();

        rclcpp::Clock clock(RCL_ROS_TIME);
        raw_dmu_data_.header.stamp = clock.now();
        raw_dmu_data_.header.frame_id = frame_id_;
        imu_raw_.header.stamp = raw_dmu_data_.header.stamp;
        imu_raw_.header.frame_id = frame_id_;

        geometry_msgs::msg::Pose pose_msg;

        imu_pose_stamped_.header.stamp = clock.now();
        imu_pose_stamped_.header.frame_id = frame_id_;
        pose_msg.position.x = 0.0;
        pose_msg.position.y = 0.0;
        pose_msg.position.z = 0.0;
        pose_msg.orientation.x = imu_raw_.orientation.x;
        pose_msg.orientation.y = imu_raw_.orientation.y;
        pose_msg.orientation.z = imu_raw_.orientation.z;
        pose_msg.orientation.w = imu_raw_.orientation.w;
        imu_pose_stamped_.pose = pose_msg;

        tf_transform_.header.stamp = clock.now();  // ROS zaman damgası
        tf_transform_.header.frame_id = "world";                    // Üst frame (parent)
        tf_transform_.child_frame_id = "imu";                       // Alt frame (child)

        tf_transform_.transform.translation.x = 0.0;
        tf_transform_.transform.translation.y = 0.0;
        tf_transform_.transform.translation.z = 0.0;

        tf_transform_.transform.rotation.x = tf_quat.x();
        tf_transform_.transform.rotation.y = tf_quat.y();
        tf_transform_.transform.rotation.z = tf_quat.z();
        tf_transform_.transform.rotation.w = tf_quat.w();


        if (callback_)
            callback_();
    }


    void Dmu11Parser::parse_data(const uint8_t *buffer, size_t size)
    {
        static std::vector<uint8_t> internal_buffer;
        internal_buffer.insert(internal_buffer.end(), buffer, buffer + size);
        while (internal_buffer.size() >= 68)
        {
            if (internal_buffer[0] == 0x55 && internal_buffer[1] == 0xAA)
            {
                int16_t received_checksum = (internal_buffer[66] << 8) | internal_buffer[67];
                if (calculate_checksum(internal_buffer.data()) == received_checksum)
                {
                    handle_packet(internal_buffer.data());  // Yeni fonksiyon
                    internal_buffer.erase(internal_buffer.begin(), internal_buffer.begin() + 68);
                }
                else
                {
                    internal_buffer.erase(internal_buffer.begin());
                }
            }
            else
            {
                internal_buffer.erase(internal_buffer.begin());
            }
        }
    }

} // namespace DMU11

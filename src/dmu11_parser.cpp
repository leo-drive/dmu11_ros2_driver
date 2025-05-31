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
            *index += 4; // Increment index by 4 bytes
        return result;
    }

    uint16_t Dmu11Parser::to_uint16_be(const uint8_t *data, uint16_t *index)
    {
        uint32_t temp = (static_cast<uint32_t>(data[0]) << 8) |
                        (static_cast<uint32_t>(data[1]));

        uint16_t result;
        std::memcpy(&result, &temp, sizeof(uint16_t));
        if (index)
            *index += 2; // Increment index by 4 bytes
        return result;
    }

    int16_t Dmu11Parser::calculate_checksum(const std::deque<uint8_t> &packet)
    {
        uint32_t sum = 0;

        // 33 adet 16-bit word'ü big-endian olarak topla
        for (size_t i = 0; i < 66; i += 2)
        {
            uint16_t word = (static_cast<uint16_t>(packet[i]) << 8) | packet[i + 1];
            sum += word;
        }

        // 16-bit ile sınırla
        sum &= 0xFFFF;

        // Two's complement (negate)
        uint16_t checksum = (~sum + 1) & 0xFFFF;

        return static_cast<int16_t>(checksum); // Gerekirse signed dön
    }

    void Dmu11Parser::handle_byte(uint8_t byte)
    {
        sync_window.push_back(byte);
        if (sync_window.size() == 68)
        {
            if (sync_window[0] == 0x55 && sync_window[1] == 0xAA)
            {
                // Check if the checksum matches the last two bytes of the packet(big endian)
                int16_t received_checksum = (sync_window[66] << 8) | sync_window[67];
                if (calculate_checksum(sync_window) == received_checksum)
                {
                    uint16_t index = 2;
                    rclcpp::Clock clock(RCL_ROS_TIME);
                    raw_dmu_data_.header.stamp = clock.now();
                    raw_dmu_data_.header.frame_id = frame_id_;

                    imu_raw_.header.stamp = clock.now();
                    imu_raw_.header.frame_id = frame_id_;

                    uint8_t *ptr = &sync_window[0];
                    raw_dmu_data_.msg_count = to_uint16_be(ptr + index, &index);

                    raw_dmu_data_.angular_rate.x = to_float_be(ptr + index, &index);
                    raw_dmu_data_.linear_acceleration.x = to_float_be(ptr + index, &index);

                    raw_dmu_data_.linear_acceleration.y = to_float_be(ptr + index, &index);
                    raw_dmu_data_.angular_rate.y = to_float_be(ptr + index, &index);

                    raw_dmu_data_.angular_rate.z = to_float_be(ptr + index, &index);
                    raw_dmu_data_.linear_acceleration.z = to_float_be(ptr + index, &index);

                    imu_raw_.linear_acceleration.x = raw_dmu_data_.linear_acceleration.x * gravity_constant; // We multiply by g_ to convert from g's into m/s^2
                    imu_raw_.linear_acceleration.y = raw_dmu_data_.linear_acceleration.y * gravity_constant;
                    imu_raw_.linear_acceleration.z = raw_dmu_data_.linear_acceleration.z * gravity_constant;

                    imu_raw_.angular_velocity.x = raw_dmu_data_.angular_rate.x * M_PI / 180; //Convert to rad/s
                    imu_raw_.angular_velocity.y = raw_dmu_data_.angular_rate.y * M_PI / 180;
                    imu_raw_.angular_velocity.z = raw_dmu_data_.angular_rate.z * M_PI / 180;

                    index += 4; // Skip the next 4 bytes (reserved for future use)
                    raw_dmu_data_.average_imu_temp = to_float_be(ptr + index, &index);

                    raw_dmu_data_.delta_theta.x = to_float_be(ptr + index, &index);
                    raw_dmu_data_.delta_velocity.x = to_float_be(ptr + index, &index);

                    raw_dmu_data_.delta_velocity.y = to_float_be(ptr + index, &index);
                    raw_dmu_data_.delta_theta.y = to_float_be(ptr + index, &index);

                    raw_dmu_data_.delta_theta.z = to_float_be(ptr + index, &index);
                    raw_dmu_data_.delta_velocity.z = to_float_be(ptr + index, &index);

                    raw_dmu_data_.system_startup_flags = to_uint16_be(ptr + index, &index);
                    raw_dmu_data_.system_operat_flags = to_uint16_be(ptr + index, &index);

                    double roll = 0, pitch = 0, yaw = 0;
                    roll += raw_dmu_data_.delta_theta.x * M_PI / 180;
                    pitch += raw_dmu_data_.delta_theta.y * M_PI / 180;
                    yaw += raw_dmu_data_.delta_theta.z * M_PI / 180;

                    // tf2 quaternion oluştur
                    tf2::Quaternion tf_quat;
                    tf_quat.setRPY(roll, pitch, yaw);
                
                    imu_raw_.orientation.x = tf_quat.x();
                    imu_raw_.orientation.y = tf_quat.y();
                    imu_raw_.orientation.z = tf_quat.z();
                    imu_raw_.orientation.w = tf_quat.w();
                    // Call the callback function if set
                    if (callback_)
                    {
                        callback_();
                    }
                }

                sync_window.clear();
            }
            else
            {
                sync_window.pop_front();
            }
        }
    }

    void Dmu11Parser::parse_data(const uint8_t *buffer, size_t size)
    {
        for (size_t i = 0; i < size; ++i)
        {
            handle_byte(buffer[i]);
        }
    }

} // namespace DMU11

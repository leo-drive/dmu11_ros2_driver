#ifndef DMU11_PARSER_HPP_
#define DMU11_PARSER_HPP_

#include <string>
#include <optional>
#include <vector>
#include <functional>
#include <dmu11_ros2_driver/msg/dmu_raw.hpp>
#include <deque>
#include <sensor_msgs/msg/imu.hpp>

namespace DMU11
{
    class Dmu11Parser
    {
    public:
        Dmu11Parser();
        Dmu11Parser(std::function<void()> callback, const std::string &frame_id = "dmu11");
        ~Dmu11Parser() = default;
        void set_callback(std::function<void()> callback);
        void set_frame_id(const std::string &frame_id);
        // Function to parse the raw data
        void parse_data(const uint8_t *buffer, size_t size);
        // Function to get the parsed data
        const dmu11_ros2_driver::msg::DmuRaw &get_dmu_raw_data() const { return raw_dmu_data_; }
        const sensor_msgs::msg::Imu &get_imu_data() const { return imu_raw_; }

    private:
        // Function to convert 4 bytes to a float in big-endian format
        static float to_float_be(const uint8_t *data, uint16_t *index = nullptr);
        // Function to convert 2 bytes to a uint16_t in big-endian format
        static uint16_t to_uint16_be(const uint8_t *data, uint16_t *index = nullptr);

        std::deque<uint8_t> sync_window; // For sliding sync search

        std::string frame_id_;
        // Callback function to process parsed data
        std::function<void()> callback_;
        
        void handle_byte(uint8_t byte);

        static int16_t calculate_checksum(const std::deque<uint8_t> &packet);

        std::vector<uint8_t> current_packet_;

        dmu11_ros2_driver::msg::DmuRaw raw_dmu_data_;
        sensor_msgs::msg::Imu imu_raw_;
    };

} // namespace DMU11

#endif // DMU11_PARSER_HPP_
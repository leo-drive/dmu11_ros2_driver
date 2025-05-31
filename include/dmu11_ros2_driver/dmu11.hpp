#ifndef DMU11_HPP_
#define DMU11_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "serial_port.h"
#include "dmu11_parser.hpp"
#include <dmu11_ros2_driver/msg/dmu_raw.hpp>

namespace DMU11
{

    class Dmu11Receiver : public rclcpp::Node
    {
    public:
        explicit Dmu11Receiver(const rclcpp::NodeOptions &options);
        ~Dmu11Receiver() = default;

    private:
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<dmu11_ros2_driver::msg::DmuRaw>::SharedPtr dmu11_raw_pub_;

        std::string serial_port_;
        std::string frame_id_;
        int working_frequency_;

        void timer_callback();

        std::shared_ptr<SerialPort> serial_port_ptr_;
        std::shared_ptr<Dmu11Parser> dmu11_parser_ptr_;

        rclcpp::TimerBase::SharedPtr timer_;

        void new_frame_received();
    };

} // namespace DMU11

#endif // DMU11__DMU11_HPP_
#ifndef RTSP_RECEIVER_COMPONENT_H
#define RTSP_RECEIVER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"
#include "usb_receiver/UsbReceiver.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace ros2_videostreamer
{
    class UsbReceiverNode : public rclcpp::Node
    {
    public:
        UsbReceiverNode();
        ~UsbReceiverNode();

    private:
        UsbReceiver receiver_;
        std::string   uri_;

		rmw_qos_profile_t image_pub_qos_profile_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    };

}// namespace read_rtsp

#endif //RTSP_RECEIVER_COMPONENT_H 



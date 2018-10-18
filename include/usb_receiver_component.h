#ifndef USB_RECEIVER_COMPONENT_H
#define USB_RECEIVER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"
#include "usb_receiver/UsbReceiver.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace ros2_videostreamer
{
    class UsbReceiverNode : public rclcpp::Node
    {
    public:
        UsbReceiverNode();
        ~UsbReceiverNode();

    private:

        void switch_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response> response);

        UsbReceiver                             receiver_;
        std::string                             uri_;

		rmw_qos_profile_t                       image_pub_qos_profile_;
        rmw_qos_profile_t                       switch_qos_profile_;

        std::string                             param_switch_service_name_;
        bool                                    switch_on_;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                       image_pub_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                          switch_service_;
    };

}// namespace read_rtsp

#endif //USB_RECEIVER_COMPONENT_H 



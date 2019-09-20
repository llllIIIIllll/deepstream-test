#ifndef USB_RECEIVER_COMPONENT_H
#define USB_RECEIVER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"
#include "usb_receiver/UsbReceiver.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

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

        void timer_check_alive_callback();
        void wait_count_subscribers();

        UsbReceiver                             receiver_;
        std::string                             uri_;
        std::string                             param_switch_service_name_;
        std::string                             param_usb_uri_;
        std::string                             param_camera_info_;
        bool                                    switch_on_;        
        bool                                    param_image_display_;
        bool                                    param_verbose;
        int                                     timer_check_alive_callback_interval = 3;

        std::vector<double>                     camera_info_message_;

        rclcpp::TimerBase::SharedPtr                                                timer_;
        rclcpp::TimerBase::SharedPtr                                                timer_check_alive_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                       image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr                  camera_info_pub_;
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                          switch_service_;

    };

}// namespace read_rtsp

#endif //USB_RECEIVER_COMPONENT_H 



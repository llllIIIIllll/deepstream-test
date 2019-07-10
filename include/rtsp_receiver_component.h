#ifndef RTSP_RECEIVER_COMPONENT_H
#define RTSP_RECEIVER_COMPONENT_H

#include "rclcpp/rclcpp.hpp"
#include "rtsp_receiver/RtspReceiver.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace ros2_videostreamer
{
    class RtspReceiverNode : public rclcpp::Node
    {
    public:
        RtspReceiverNode();
        ~RtspReceiverNode();

    private:

        void switch_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
                            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                            const std::shared_ptr<std_srvs::srv::SetBool::Response> response);
        void topic_rtsp_uri_callback_shared(const std_msgs::msg::String::SharedPtr msg);

        void timer_check_alive_callback();
        void timer_stream_controller_callback();
        
        void start_stream();
        void stop_stream();
        
        std::shared_ptr<RtspReceiver>           receiver_;
        std::string                             uri_;

        std::string                             param_switch_service_name_;
        std::string                             param_rtsp_uri_;
        std::string                             param_rtsp_uri_topic_;

        bool                                    stream_alive_;
        bool                                    stream_restart_;
        bool                                    switch_on_;
        bool                                    turn_on_or_off_;
        bool                                    param_image_display_;
        bool                                    param_verbose_;
        bool                                    param_auto_start_;
        
        std::chrono::system_clock::time_point   turn_off_count_start_;

        rclcpp::TimerBase::SharedPtr                                                timer_check_alive_;
        rclcpp::TimerBase::SharedPtr                                                timer_stream_controller_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr                       image_pub_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr                      rtsp_uri_;  
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr                          switch_service_;
    };

}// namespace read_rtsp

#endif //RTSP_RECEIVER_COMPONENT_H 



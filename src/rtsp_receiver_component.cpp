#include "rtsp_receiver_component.h"
#define NODE_NAME "rtsp"
#define NAME_SPACE ""
namespace ros2_videostreamer
{
	RtspReceiverNode::RtspReceiverNode()
		:Node(NODE_NAME,
        NAME_SPACE,
        rclcpp::NodeOptions(
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
            )
        )
	{
		param_switch_service_name_ = "switch_on";
        param_rtsp_uri_ = "rtsp://192.168.1.21:554/ch4";
        param_rtsp_uri_topic_ = "rtsp_uri";

		this->switch_on_ = true;

        this->get_parameter_or("param_rtsp_uri", param_rtsp_uri_,param_rtsp_uri_);
        // this->get_parameter_or("param_rtsp_uri_topic", param_rtsp_uri_topic_,param_rtsp_uri_topic_);

        auto image_pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto image_sub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        // auto switch_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", image_pub_qos_profile);

		this->receiver_.data._height = 720;
		this->receiver_.data._width = 1280;
		this->receiver_.data.image_pub_ = image_pub_;
		//this->uri_ = "rtsp://192.168.1.242:554/live";
		this->uri_ = this->param_rtsp_uri_;
		this->receiver_.setUri(this->uri_);

        auto switch_cb = std::bind(&RtspReceiverNode::switch_service_callback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            param_switch_service_name_, switch_cb);
            
        auto rtsp_cb = std::bind(&RtspReceiverNode::topic_rtsp_uri_callback_shared, this, std::placeholders::_1);
        rtsp_uri_ = this->create_subscription<std_msgs::msg::String>(
            param_rtsp_uri_topic_, image_sub_qos_profile, rtsp_cb);

        if(switch_on_)
        {
			this->receiver_.start();
        }
	}

    void RtspReceiverNode::switch_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // RCLCPP_INFO(this->get_logger(), "in switch CB");
        if(switch_on_ && !request->data )
        {
			switch_on_ = false;
			this->receiver_.stop();
        }
        else if(!switch_on_ && request->data)
        {
			switch_on_ = true;
			this->receiver_.start();
        }

        RESERVE(request_header);
        RESERVE(request);
        RESERVE(response);
    }
    
    void RtspReceiverNode::topic_rtsp_uri_callback_shared(const std_msgs::msg::String::SharedPtr msg)
    {
        if (this->param_rtsp_uri_ != msg->data)
        {
            this->param_rtsp_uri_ = msg->data;
            if (switch_on_)
            {
    			this->receiver_.stop();
                std::this_thread::sleep_for(std::chrono::seconds(1));
                this->receiver_.start();
            }
        }

    }

	RtspReceiverNode::~RtspReceiverNode()
	{}
};

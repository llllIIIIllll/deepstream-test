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
        this->receiver_  = std::make_shared<RtspReceiver>();
        
		param_switch_service_name_ = "switch_on";
        param_rtsp_uri_ = "rtsp://192.168.1.21:554/ch4";
        param_rtsp_uri_topic_ = "rtsp_uri";

		this->stream_restart_ = false;
        this->param_image_display_ = true;
        this->param_verbose_ = true;
        this->param_auto_start_ = true;

        this->get_parameter_or("param_rtsp_uri", param_rtsp_uri_,param_rtsp_uri_);
        this->get_parameter_or("param_rtsp_uri_topic", param_rtsp_uri_topic_,param_rtsp_uri_topic_);
        this->get_parameter_or("display", param_image_display_, param_image_display_);
        this->get_parameter_or("verbose", param_verbose_, param_verbose_);
        this->get_parameter_or("auto_start", param_auto_start_,param_auto_start_);
        
        this->switch_on_ = this->param_auto_start_;

        auto image_pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        auto image_sub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        // auto switch_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", image_pub_qos_profile);

        auto switch_cb = std::bind(&RtspReceiverNode::switch_service_callback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            param_switch_service_name_, switch_cb);
            
        auto rtsp_cb = std::bind(&RtspReceiverNode::topic_rtsp_uri_callback_shared, this, std::placeholders::_1);
        rtsp_uri_ = this->create_subscription<std_msgs::msg::String>(
            param_rtsp_uri_topic_, image_sub_qos_profile, rtsp_cb);

        this->timer_check_alive_ = this->create_wall_timer(std::chrono::seconds(5), std::bind(&RtspReceiverNode::timer_check_alive_callback, this));
        // this->timer_check_alive_->cancel();

	}

    void RtspReceiverNode::switch_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        // RCLCPP_INFO(this->get_logger(), "in switch CB");
        if(switch_on_ && !request->data )
        {
            RCLCPP_INFO(this->get_logger(), "switch off");
            this->stop_stream();
        }
        else if(!switch_on_ && request->data)
        {
            RCLCPP_INFO(this->get_logger(), "switch on");
			this->start_stream();
        }
        
        switch_on_ = request->data;
        
        RESERVE(request_header);
        RESERVE(request);
        RESERVE(response);
    }
    
    void RtspReceiverNode::topic_rtsp_uri_callback_shared(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "stream running : %s", msg->data);
        if (this->param_rtsp_uri_ != msg->data)
        {
            this->param_rtsp_uri_ = msg->data;
            if (switch_on_)
            {
                this->stop_stream();
                this->start_stream();
            }
        }

    }

	RtspReceiverNode::~RtspReceiverNode()
	{}

    void RtspReceiverNode::start_stream()
    {        
        this->receiver_->data.image_pub_ = image_pub_;
        this->receiver_->setDisplay(param_image_display_);
        this->receiver_->setVerbose(param_verbose_);
        this->receiver_->setUri(param_rtsp_uri_);
        this->receiver_->setStreamAlive(false);
        
        this->receiver_->start();
    }
    
    void RtspReceiverNode::stop_stream()
    {
        this->receiver_->stop();
    }
	
    void RtspReceiverNode::timer_check_alive_callback()
    {
        if (!this->switch_on_ || this->receiver_ == NULL)
            return;
        else if (this->switch_on_ && this->receiver_ == NULL)
            this->start_stream();

        this->stream_alive_ = this->receiver_->getStreamAlive();
        
        if (!this->stream_alive_)
        {
            this->stop_stream();
            this->start_stream();
        }
        else 
        {
            this->receiver_->setStreamAlive(false);
        }
    }
    
};

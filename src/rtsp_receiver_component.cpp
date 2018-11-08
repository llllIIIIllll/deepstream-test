#include "rtsp_receiver_component.h"

namespace ros2_videostreamer
{
	RtspReceiverNode::RtspReceiverNode()
		: Node("rtsp", "", true)
	{
		param_switch_service_name_ = "switch_on";
        param_rtsp_uri_ = "rtsp://admin:admin@192.168.1.108:554/cam/realmonitor?channel=1&subtype=2";

		this->switch_on_ = true;

        this->get_parameter_or("param_rtsp_uri", param_rtsp_uri_,param_rtsp_uri_);

        image_pub_qos_profile_ = rmw_qos_profile_default;
        switch_qos_profile_ = rmw_qos_profile_default;

		switch_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        switch_qos_profile_.depth = 10;
        switch_qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;


		image_pub_qos_profile_.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		image_pub_qos_profile_.depth = 10;
		image_pub_qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
		//image_pub_qos_profile_.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", image_pub_qos_profile_);

		this->receiver_.data._height = 720;
		this->receiver_.data._width = 1280;
		this->receiver_.data.image_pub_ = image_pub_;
		//this->uri_ = "rtsp://192.168.1.242:554/live";
		this->uri_ = this->param_rtsp_uri_;
		this->receiver_.setUri(this->uri_);

        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            param_switch_service_name_, std::bind(&RtspReceiverNode::switch_service_callback, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),switch_qos_profile_);

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

	RtspReceiverNode::~RtspReceiverNode()
	{}
};

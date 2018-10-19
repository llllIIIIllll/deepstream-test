#include "usb_receiver_component.h"

namespace ros2_videostreamer
{
	UsbReceiverNode::UsbReceiverNode()
		: Node("usb", "", true)
	{
        param_switch_service_name_ = "/usb_node/switch_on";
		this->switch_on_ = true;
        param_usb_uri_ = "/dev/video0";

        this->get_parameter_or("param_usb_uri", param_usb_uri_,param_usb_uri_);
        std::cout << param_usb_uri_ << std::endl;

        image_pub_qos_profile_ = rmw_qos_profile_default;
        switch_qos_profile_ = rmw_qos_profile_default;
		
		switch_qos_profile_.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        switch_qos_profile_.depth = 10;
        switch_qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;


        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", image_pub_qos_profile_);

		this->receiver_.data.image_pub_ = image_pub_;
        this->uri_ = param_usb_uri_;
		this->receiver_.setUri(this->uri_);
	

		this->receiver_.set_resulation(640, 480);
		this->receiver_.set_format("RGB");
		this->receiver_.set_fps(30);

        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            param_switch_service_name_, std::bind(&UsbReceiverNode::switch_service_callback, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3),switch_qos_profile_);

        if(switch_on_)
        {
			this->receiver_.start();
        }
        
	}

    void UsbReceiverNode::switch_service_callback(const std::shared_ptr<rmw_request_id_t> request_header,
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

	UsbReceiverNode::~UsbReceiverNode()
	{}
};

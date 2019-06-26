#include "usb_receiver_component.h"
#define NODE_NAME "usb"
#define NAME_SPACE ""
namespace ros2_videostreamer
{
	UsbReceiverNode::UsbReceiverNode()
		:Node(NODE_NAME,
        NAME_SPACE,
        rclcpp::NodeOptions(
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
            )
        )
	{
        param_switch_service_name_ = "/usb_node/switch_on";
		this->switch_on_ = true;
        param_usb_uri_ = "/dev/video0";

        // this->get_parameter_or("param_usb_uri", param_usb_uri_,param_usb_uri_);
        // this->get_parameter_or("param_camera_info", param_camera_info_, param_camera_info_);

        {
            // parse camera_info
            double k;
            std::stringstream iss(param_camera_info_);
            while (iss >> k)
                camera_info_message_.push_back(k);
        }

        auto image_pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
        // auto switch_qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", image_pub_qos_profile);

        camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
            "camera_info", image_pub_qos_profile);

        auto timer_callback =
            [this]() -> void {
                sensor_msgs::msg::CameraInfo message;
                message.k[0] = camera_info_message_[0];
                message.k[2] = camera_info_message_[1];
                message.k[4] = camera_info_message_[2];
                message.k[5] = camera_info_message_[3];
                message.k[8] = camera_info_message_[4];

                this->camera_info_pub_->publish(message);
        };

        timer_ = this->create_wall_timer(std::chrono::microseconds(500), timer_callback);
        // cancel immediately to prevent it running the first time.
        timer_->cancel();

		this->receiver_.data.image_pub_ = image_pub_;
        this->uri_ = param_usb_uri_;
		this->receiver_.setUri(this->uri_);
	

		this->receiver_.set_resulation(640, 480);
		this->receiver_.set_format("RGB");
		this->receiver_.set_fps(30);

        switch_service_ = this->create_service<std_srvs::srv::SetBool>(
            param_switch_service_name_, std::bind(&UsbReceiverNode::switch_service_callback, this, std::placeholders::_1,std::placeholders::_2,std::placeholders::_3));

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

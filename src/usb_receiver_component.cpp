#include "usb_receiver_component.h"

namespace ros2_videostreamer
{
	UsbReceiverNode::UsbReceiverNode()
		: Node("usb", "", true)
	{
        image_pub_qos_profile_ = rmw_qos_profile_default;

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image", image_pub_qos_profile_);

		this->receiver_.data.image_pub_ = image_pub_;
		this->uri_ = "/dev/video0";
		this->receiver_.setUri(this->uri_);
	

		this->receiver_.set_resulation(640, 480);
		this->receiver_.set_format("RGB");
		this->receiver_.set_fps(30);

		this->receiver_.start();
	}

	UsbReceiverNode::~UsbReceiverNode()
	{}
};

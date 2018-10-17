#include "rtsp_receiver_component.h"

namespace ros2_videostreamer
{
	RtspReceiverNode::RtspReceiverNode()
		: Node("rtsp", "", true)
	{
        image_pub_qos_profile_ = rmw_qos_profile_default;

		image_pub_qos_profile_.history=RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		image_pub_qos_profile_.depth = 100;
		//image_pub_qos_profile_.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
		//image_pub_qos_profile_.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "image_raw", image_pub_qos_profile_);

		this->receiver_.data._height = 720;
		this->receiver_.data._width = 1280;
		this->receiver_.data.image_pub_ = image_pub_;
		//this->uri_ = "rtsp://192.168.1.242:554/live";
		this->uri_ = "rtsp://admin:admin@192.168.1.108:554/cam/realmonitor?channel=1&subtype=2";
		this->receiver_.setUri(this->uri_);

		this->receiver_.start();
	}

	RtspReceiverNode::~RtspReceiverNode()
	{}
};

#ifndef __USBRECEIVER_HPP__
#define __USBRECEIVER_HPP__

#include "Receiver.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// the stream path
/*
 * v4l2src --> capsfilter --> appsink(new-sample)
 * 										 |
 * 										 --> to ros publish
*/

// gstreamer data
typedef struct _CustomData
{
	GstElement *source;
	GstElement *capsfilter;
	GstElement *sink;

	int _height;
	int _width;
	int _fps;

	std::string _format;

	int _brightness;					  // min=-64 max=64 step=1 default=0 value=0
	int _contrast;						  // min=0 max=95 step=1 default=0 value=0
	int _saturation;					  // min=0 max=100 step=1 default=64 value=64
	int _hue;							  // min=-2000 max=2000 step=1 default=0 value=0
	bool _white_balance_temperature_auto; // default=1 value=1
	int _gamma;							  // min=100 max=300 step=1 default=100 value=100
	int _power_line_frequency;			  // min=0 max=2 default=1 value=1
	int _white_balance_temperature;		  // min=2800 max=6500 step=1 default=4600
										  // value=4600 flags=inactive
	int _sharpness;						  // min=1 max=7 step=1 default=2 value=7
	int _backlight_compensation;		  // min=0 max=3 step=1 default=3 value=3
	int _exposure_auto;					  // min=0 max=3 default=3 value=1
	int _exposure_absolute;				  // min=10 max=626 step=1 default=156 value=156
	bool _exposure_auto_priority;		  // default=0 value=1

	rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

} CustomData;

static void new_sample(GstElement *sink, CustomData *data);

class UsbReceiver : public Receiver 
{
public:
	explicit UsbReceiver();
	~UsbReceiver();

	CustomData data;
	GstElement *pipeline;

	void start();

	void set_data_elements(GstCaps *caps, GstStructure *extraCtrls);

	void set_resulation(int width, int height)
	{
		data._height = height;
		data._width = width;
	}
	void set_fps(int fps) { data._fps = fps; }
	void set_format(std::string format) { data._format = format; }
};

#endif // __USBRECEIVER_HPP__


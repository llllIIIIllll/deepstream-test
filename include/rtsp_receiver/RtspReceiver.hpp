#ifndef __RTSPRECEIVER_HPP__
#define __RTSPRECEIVER_HPP__

#include "Receiver.hpp"
#include "common.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// the stream path
/* rtspsrc --> rtph264depay --> h264parse --> avdec_h264 --> capsfilter --> appsink(new-samplw)
 * 																				|
 * 																				--> ros publish
*/

// gstreamer data
typedef struct _CustomData
{
	GstElement *source;
	GstElement *rtppay;
	GstElement *parse;
	GstElement *identity;
	GstElement *filter1;
	GstElement *decodebin;

GstElement *decoder;
GstElement *nvvidconv;
GstElement *vidconv;

GstElement *nvosd;
GstElement *transform;
GstElement *streammux;

	GstElement *capsfilter;
	GstElement *sink;

GstCaps *caps;

	int _height;
	int _width;
	int _fps;
	bool _image_display;
	bool _verbose;
	bool _stream_alive;

	std::string _format;
	uint32_t _timestamp;

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
	std::string host_cpu_;

} CustomData;

// void new_sample(GstAppSink *sink, CustomData *data);
void handoff(GstElement *sink, GstBuffer* buffer,CustomData *data);

class RtspReceiver : public Receiver 
{
public:
	explicit RtspReceiver();
	~RtspReceiver();

	CustomData data;
	// GstElement *pipeline;
GstPipeline *pipeline = nullptr;

	cv::Mat* output_;

	void start();
	bool getStreamAlive();

	void set_resulation(int width, int height)
	{
		data._height = height;
		data._width = width;
	}
	void set_fps(int fps) { data._fps = fps; }
	void set_format(std::string format) { data._format = format; }
};

#endif // __RTSPRECEIVER_HPP__


#include "usb_receiver/UsbReceiver.hpp"

/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */
std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) 
  {
    case CV_8UC1 : return "mono8";
    case CV_8UC3 : return "bgr8";
    case CV_16SC1: return "mono16";
    case CV_8UC4 : return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(const cv::Mat & frame,
			   						size_t frame_id, 
									sensor_msgs::msg::Image::SharedPtr msg)
{
  // copy cv information into ros message
  msg->height = frame.rows;
  msg->width = frame.cols;
  msg->encoding = mat_type2encoding(frame.type());
  msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
  size_t size = frame.step * frame.rows;
  msg->data.resize(size);
  memcpy(&msg->data[0], frame.data, size);
  msg->header.frame_id = std::to_string(frame_id);
}

UsbReceiver::UsbReceiver() : Receiver()
{
	// default settting
	data._height = 480;
	data._width  = 640;
	data._fps    = 30;
	data._format = "BGR";
}

UsbReceiver::~UsbReceiver() {}

void UsbReceiver::set_data_elements(GstCaps *caps, GstStructure *extraCtrls)
{
	// resulation format fps
	caps = gst_caps_new_simple(
		"video/x-raw",
		"format"     , G_TYPE_STRING    , data._format.c_str(),
		"width"      , G_TYPE_INT       , data._width         ,
		"height"     , G_TYPE_INT       , data._height        ,
		"framerate"  , GST_TYPE_FRACTION, data._fps           , 1,
		NULL);
	// v4l2src extra-control
	// g_object_set(data.source, "extra-controls",
	// "c,brightness=50,exposure_absolute=100", NULL);
	// Setup camera controls

	try
	{
		g_object_set(data.source        , "device"      , _uri.c_str(), NULL);
		g_object_set(data.capsfilter    , "caps"        , caps        , NULL);
		g_object_set(G_OBJECT(data.sink), "sync"        , TRUE        , NULL);
		g_object_set(data.sink          , "emit-signals", TRUE        , NULL);
	}
	catch (const std::exception &ex)
	{
		g_printerr("%s\n", ex.what());
		throw;
	}

	// free caps and extraCtrls
	gst_caps_unref(caps);
	gst_structure_free(extraCtrls);
}

void UsbReceiver::start()
{
	if (_uri.empty())
	{
		g_print("UsbReceiver::start() failed because URI is not specified");
		return;
	}
	if (_running)
	{
		g_print("Already running!");
		return;
	}
	_starting       = true;

	bool running    = false;
	bool pipelineUp = false;

	GstBus *bus;
	GstMessage *msg;
	GstStateChangeReturn ret;

	do
	{
		// create elements
		data.source     = gst_element_factory_make("v4l2src", "source");
		data.capsfilter = gst_element_factory_make("capsfilter", "capsfilter");
		data.sink       = gst_element_factory_make("appsink", "sink");

		GstCaps *caps            = NULL;
		GstStructure *extraCtrls = NULL;

		set_data_elements(caps, extraCtrls);

		// create the empty pipeline
		_pipeline = gst_pipeline_new("test-pipeline");

		if (!_pipeline || !data.source || !data.capsfilter || !data.sink)
		{
			g_printerr("Not all elements could be created.\n");
		}

		// build the pipeline
		gst_bin_add_many(GST_BIN(_pipeline), data.source, data.capsfilter,
						 data.sink, NULL);
		if (!gst_element_link_many(data.source, data.capsfilter, data.sink, NULL))
		{
			g_printerr("Elements could not be linked .\n");
			gst_object_unref(_pipeline);
		}

		// signal
		g_signal_connect(data.sink, "new-sample", G_CALLBACK(new_sample), &data);

		// start playing
		ret = gst_element_set_state(_pipeline, GST_STATE_PLAYING);
		if (ret == GST_STATE_CHANGE_FAILURE)
		{
			g_printerr("Unable to set the pipeline to the playing state.\n");
			gst_object_unref(_pipeline);
		}

		if ((bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline))) != NULL)
		{
			gst_bus_enable_sync_message_emission(bus);
			g_signal_connect(bus, "sync-message", G_CALLBACK(_onBusMessage), this);
			gst_object_unref(bus);
			bus = NULL;
		}

		GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(_pipeline), GST_DEBUG_GRAPH_SHOW_ALL,
								  "pipeline-paused");
		running = gst_element_set_state(_pipeline, GST_STATE_PLAYING) !=
				  GST_STATE_CHANGE_FAILURE;
	} while (0);

	if (!running)
	{
		g_print("UsbReceiver::start() failed");

		// In newer versions, the pipeline will clean up all references that are
		// added to it
		if (_pipeline != NULL)
		{
			gst_object_unref(_pipeline);
			_pipeline = NULL;
		}

		auto unref = [&](GstElement *a) {
			g_print("%s: ", gst_element_get_name(a));
			if (a != NULL)
			{
				gst_object_unref(a);
			}
			a = NULL;
			g_print("unref success\n");
		};
		unref(data.source);
		unref(data.capsfilter);
		unref(data.sink);

		// If we failed before adding items to the pipeline, then clean up

		_running = false;
	}
	else
	{
		GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(_pipeline), GST_DEBUG_GRAPH_SHOW_ALL,
								  "pipeline-playing");
		_running = true;
		g_print("Running\n");
	}
	_starting = false;
}

/* The appsink has received a buffer */
static void new_sample(GstElement *sink, CustomData *data)
{
	GstSample *sample;
	GstBuffer *buffer;
	GstMapInfo map;

	//g_print("Callback\n");

	/* Retrieve the buffer */
	g_signal_emit_by_name(sink, "pull-sample", &sample);
	if (sample)
	{
		// get buffer
		buffer = gst_sample_get_buffer(sample);
		gst_buffer_map(buffer, &map, GST_MAP_READ);

		cv::Mat mRGB;
		if (data->_format == "I420")
		{
			cv::Mat t = cv::Mat(data->_height + data->_height / 2, data->_width,
								CV_8UC1, (void *)map.data);
			cvtColor(t, mRGB, CV_YUV420p2RGB);
		}
		else if (data->_format == "RGB")
		{
			cv::Mat t =
				cv::Mat(data->_height, data->_width, CV_8UC3, (void *)map.data);
			cvtColor(t, mRGB, CV_BGR2RGB);
		}
		else
		{
			mRGB = cv::Mat(data->_height, data->_width, CV_8UC3, (void *)map.data);
		}

		sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image());

		//cv::imshow("usb", mRGB);
		//cv::waitKey(1);

		convert_frame_to_message(mRGB, 0, msg);

		data->image_pub_->publish(msg);

		gst_buffer_unmap(buffer, &map);
		gst_sample_unref(sample);
	}
}

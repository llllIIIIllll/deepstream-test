#include "host_cpu.h"
#include "rtsp_receiver/RtspReceiver.hpp"
#include <chrono>


// TODO: delete unnecessary component
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
        default      : throw std::runtime_error("Unsupported encoding type");
  	}
}

/// Convert an OpenCV matrix (cv::Mat) to a ROS Image message.
/**
 * \param[in] frame The OpenCV matrix/image to convert.
 * \param[in] frame_id ID for the ROS message.
 * \param[out] Allocated shared pointer for the ROS Image message.
 */
void convert_frame_to_message(
  const cv::Mat & frame, size_t frame_id, sensor_msgs::msg::Image::SharedPtr msg)
{
  	// copy cv information into ros message
	msg->height          = frame.rows;
	msg->width           = frame.cols;
	msg->encoding        = mat_type2encoding(frame.type());
	msg->step            = static_cast<sensor_msgs::msg::Image::_step_type>(frame.step);
	size_t size          = frame.step * frame.rows;

	msg->data.resize(size);
	memcpy(&msg->data[0], frame.data, size);
	msg->header.frame_id = std::to_string(frame_id);
  	std::chrono::nanoseconds now = std::chrono::high_resolution_clock::now().time_since_epoch();
    msg->header.stamp.sec = static_cast<builtin_interfaces::msg::Time::_sec_type>(now.count() / 1000000000);
    msg->header.stamp.nanosec = now.count() % 1000000000;
}

static void on_pad_added (GstElement *element, GstPad *pad, gpointer data)
{
    GstPad *sinkpad;
    GstElement *decoder = (GstElement *) data;
    /* We can now link this pad with the rtsp-decoder sink pad */
    g_print ("Dynamic pad created, linking source/demuxer\n");
    sinkpad = gst_element_get_static_pad (decoder, "sink");
    gst_pad_link (pad, sinkpad);
    gst_object_unref (sinkpad);
}

static void cb_new_rtspsrc_pad(GstElement *element,GstPad*pad,gpointer  data)
{
	gchar       * name;
	GstCaps     * p_caps;
	gchar       * description;
	GstElement  * p_rtph264depay;

	name = gst_pad_get_name(pad);
	g_print("A new pad %s was created\n", name);
	// here, you would setup a new pad link for the newly created pad
	p_caps = gst_pad_get_pad_template_caps (pad);

	description = gst_caps_to_string(p_caps);
	printf("%s\n",p_caps,", ",description,"\n");
	g_free(description);

	p_rtph264depay = GST_ELEMENT(data);

	// try to link the pads then ...
	if(!gst_element_link_pads(element, name, p_rtph264depay, "sink"))
	{
	    printf("Failed to link elements 3\n");
	}

	g_free(name);
}

RtspReceiver::RtspReceiver() : Receiver()
{
}

RtspReceiver::~RtspReceiver() {}

void RtspReceiver::start()
{
	if (_uri.empty())
	{
		g_print("RtspReceiver::start() failed because URI is not specified");
		return;
	}
	if (_running)
	{
		g_print("Already running!");
		return;
	}
	_starting = true;

	bool running    = false;
	bool pipelineUp = false;

	GstBus                     * bus;
	GstMessage                 * msg;
	GstStateChangeReturn 		 ret;

	do
	{
		// TODO: make code consice
		_pipeline = gst_pipeline_new("Rtsp pipeline");

		data.source    = gst_element_factory_make( "rtspsrc"     , "source");
		data.rtppay    = gst_element_factory_make( "rtph265depay", "depayl");
		data.parse     = gst_element_factory_make( "h265parse"   , "parse" );
		//data.filter1   = gst_element_factory_make( "capsfilter"  , "filter");
		
		if (CMAKE_HOST_SYSTEM_PROCESSOR == "aarch64")
		{
			data.decodebin = gst_element_factory_make( "omxh265dec"  , "decode");
		}
		else
		{
			data.decodebin = gst_element_factory_make( "avdec_h265"  , "decode");
		}

		data.sink      = gst_element_factory_make( "appsink"     , "sink"  );

		// set up link
		g_object_set (G_OBJECT (data.source)   , "latency"    , 50         , NULL);
		g_object_set (G_OBJECT (data.sink)     , "sync"       , FALSE       ,"drop", TRUE, NULL);
		g_object_set (G_OBJECT (data.decodebin), "max-threads", 6           , NULL);
		g_object_set(GST_OBJECT(data.source)   , "location"   , _uri.c_str(), NULL);

		// connect appsink to caps
		//GstCaps* filtercaps = gst_caps_from_string("application/x-rtp");
		//g_object_set (G_OBJECT (data.filter1), "caps", filtercaps, NULL);
		//gst_caps_unref(filtercaps);
  		g_object_set (data.sink, "emit-signals", TRUE, NULL);

		gst_bin_add_many (GST_BIN (_pipeline), data.source
		                                     , data.rtppay
		                                     , NULL);
		// listen for newly created pads
		g_signal_connect(data.source, "pad-added", G_CALLBACK(cb_new_rtspsrc_pad), data.rtppay);

		gst_bin_add_many (GST_BIN (_pipeline), data.parse, NULL);
		if(!gst_element_link(data.rtppay,data.parse))
		{
			printf("\nNOPE\n");
		}
		gst_bin_add_many (GST_BIN (_pipeline), data.decodebin
		                                     , data.sink
		                                     , NULL);
		
		if(!gst_element_link_many(data.parse, data.decodebin, data.sink, NULL))
		{
		    printf("\nFailed to link parse to sink");
		}
		
		g_signal_connect(data.rtppay, "pad-added" , G_CALLBACK(on_pad_added), data.parse);
		g_signal_connect(data.sink  , "new-sample", G_CALLBACK(new_sample), &data);

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
		g_print("RtspReceiver::start() failed");

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
		unref(data.rtppay);
		unref(data.parse);
		unref(data.filter1);
		unref(data.decodebin);
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
	GstSample   *sample;
	GstBuffer   *buffer;
	GstMapInfo  map;
    gboolean    res;
	/* Retrieve the buffer */
	g_signal_emit_by_name(sink, "pull-sample", &sample);
	//auto start = std::chrono::system_clock::now();
	if (sample)
	{
		// get picture format
		GstCaps *caps;
		GstStructure *s;
    	caps = gst_sample_get_caps (sample);
    	if (!caps) 
		{
    	  g_print ("could not get snapshot format\n");
    	  exit (-1);
    	}
    	s = gst_caps_get_structure (caps, 0);
    	/* we need to get the final caps on the buffer to get the size */
    	res = gst_structure_get_int (s, "width", &data->_width);
    	res |= gst_structure_get_int (s, "height", &data->_height);

		if (!res) 
		{
			g_print ("could not get snapshot dimension\n");
			exit (-1);
		}

		// get buffer
		buffer = gst_sample_get_buffer(sample);
		gst_buffer_map(buffer, &map, GST_MAP_READ);

		cv::Mat mRGB;
		sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image());

		bool use_rgb = true;
		if (use_rgb)
		{
			cv::Mat t = cv::Mat(data->_height + data->_height / 2 , data->_width,
								CV_8UC1, (void *)map.data);
			cvtColor(t, mRGB, CV_YUV420p2BGR);
			convert_frame_to_message(mRGB, 10, msg);
			//cv::imshow("usb", mRGB);
		}
		else
		{

			cv::Mat t = cv::Mat(data->_height /*+ data->_height / 2*/ , data->_width,
								CV_8UC1, (void *)map.data);
			convert_frame_to_message(t, 10, msg);
			//cv::imshow("usb", t);
		}
		//cv::waitKey(1);

		// TODO: add frame_id
		data->image_pub_->publish(msg);

		gst_buffer_unmap(buffer, &map);
		gst_sample_unref(sample);
	}
	//auto end = std::chrono::system_clock::now();
	//std::chrono::duration<double> diff = end - start;
	//std::cout << "Time:  " << GST_BUFFER_PTS(buffer) << std::endl;
}

#include "host_cpu.h"
#include "rtsp_receiver/RtspReceiver.hpp"
#include <chrono>
#include <cstdlib>
#include <gst/gst.h>
#include <gst/gstinfo.h>
#include <gst/app/gstappsink.h>
#include <glib-unix.h>
#include <dlfcn.h>

#include <iostream>
#include <sstream>
#include <thread>




static int frame_count = 0;
static int sleep_count = 0;
static int eos = 0;


#define MUXER_OUTPUT_WIDTH 1920
#define MUXER_OUTPUT_HEIGHT 1080
#define MUXER_BATCH_TIMEOUT_USEC 4000000

// TODO: delete unnecessary component
/// Convert an OpenCV matrix encoding type to a string format recognized by sensor_msgs::Image.
/**
 * \param[in] mat_type The OpenCV encoding type.
 * \return A string representing the encoding type.
 */

 static std::string launch_string;
 static std::ostringstream launch_stream;

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


static void appsink_eos(GstAppSink * appsink, gpointer data)
{
    printf("app sink receive eos\n");
    eos = 1;
//    g_main_loop_quit (hpipe->loop);
}


/* The appsink has received a buffer */
static GstFlowReturn new_buffer(GstAppSink *appsink, gpointer user_data)
{

    GstSample *sample = NULL;

    g_signal_emit_by_name (appsink, "pull-sample", &sample,NULL);

    if (sample)
    {
        GstBuffer *buffer = NULL;
        GstCaps   *caps   = NULL;
        GstMapInfo map    = {0};
        int dmabuf_fd = 0;

        caps = gst_sample_get_caps (sample);
        if (!caps)
        {
            printf("could not get snapshot format\n");
        }
        gst_caps_get_structure (caps, 0);
        buffer = gst_sample_get_buffer (sample);
        gst_buffer_map (buffer, &map, GST_MAP_READ);

		cv::Mat t = cv::Mat(1080 , 1920,
								CV_8UC1, (void *)map.data);

			cv::imshow("usb", t);
			cv::waitKey(1);

        gst_buffer_unmap(buffer, &map);

        gst_sample_unref (sample);
    }
    else
    {
        g_print ("could not make snapshot\n");
    }

    return GST_FLOW_OK;
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
	RESERVE(element);

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
	// printf("%s\n",p_caps,", ",description,"\n");
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
	g_print("\nRtspReceiver::RtspReceiver()\n");
	data.host_cpu_ = (std::string)CMAKE_HOST_SYSTEM_PROCESSOR;
	std::cout << data.host_cpu_ << std::endl;
}

RtspReceiver::~RtspReceiver() { 
	stop();
	_stream_alive = false;
	_running = false;
	_starting = false;
	_stopping = false;
	_tee = nullptr;
	_pipeline = nullptr;
	_videoSink = nullptr;
	g_print("\nRtspReceiver::~RtspReceiver()\n");
 }
static void cb_new_sample_sink(GstElement *sink, cv::Mat *output) {
  GstSample *sample;
  GstBuffer *buffer;
  GstMapInfo map;
  gboolean res;

  /* Retrieve the buffer */
  g_signal_emit_by_name(G_OBJECT(sink), "pull-sample", &sample);
  if (sample) {
    // get picture format
    GstCaps *caps;
    GstStructure *s;
    caps = gst_sample_get_caps(sample);
    if (!caps) {
      g_print("could not get snapshot format\n");
      exit(-1);
    }
    s = gst_caps_get_structure(caps, 0);
    /* we need to get the final caps on the buffer to get the size */
    int width, height;
    res = gst_structure_get_int(s, "width", &width);
    res |= gst_structure_get_int(s, "height", &height);

    if (!res) {
      g_print("could not get snapshot dimension\n");
      exit(-1);
    }
    // get buffer
    buffer = gst_sample_get_buffer(sample);
    gst_buffer_map(buffer, &map, GST_MAP_READ);
    cv::Mat t = cv::Mat(height + height / 2, width, CV_8UC1, (void *)map.data);
    cv::cvtColor(t, *output, CV_YUV2BGR_NV12);

    cv::imshow("rtsp", *output);
    cv::waitKey(1);
    gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);
  }
}
void RtspReceiver::start()
{
	 output_ = new cv::Mat();

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

	RESERVE(pipelineUp);
	RESERVE(msg);

	data._image_display = this->_image_display;
	data._verbose = this->_verbose;

	do
	{

		// TODO: make code consice
		// _pipeline = gst_pipeline_new("Rtsp pipeline");

//    << "nvcamerasrc ! "
//    << "video/x-raw(memory:NVMM), width="<< w <<", height="<< h <<", framerate=30/1 ! " 
    // launch_stream
    // << "filesrc location=Bourne_Trailer.mp4 ! decodebin ! "
    // << "nvvidconv ! "
    // << "video/x-raw(memory:NVMM), format=I420, width="<< w <<", height="<< h <<" ! "
    // << "appsink name=mysink ";

    launch_string = "rtspsrc location=rtsp://admin:xijingkeji2020@192.168.1.64 latency=200 ! rtph264depay ! h264parse ! omxh264dec \
	! nvvidconv ! video/x-raw, width=(int)1920, height=(int)1080, format=(string)GRAY8 ! videoconvert ! appsink name=mysink ";
					// !  nvvidconv !  video/x-raw, width=(int)1920, height=(int)1080, format=(string)BGRx \
					// !  videoconvert 
	// launch_string = "rtspsrc location=rtsp://admin:xijingkeji2020@192.168.1.64:554 latency=200 drop-on-latency=0 ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvideoconvert ! nvdsosd ! nvegltransform ! nveglglessink";
    g_print("Using launch string: %s\n", launch_string.c_str());

    GError *error = nullptr;
    pipeline  = (GstPipeline*) gst_parse_launch(launch_string.c_str(), &error);

    GstAppSinkCallbacks callbacks = {appsink_eos, NULL, new_buffer};

    data.sink = gst_bin_get_by_name(GST_BIN(pipeline), "mysink");
    gst_app_sink_set_callbacks (GST_APP_SINK(data.sink), &callbacks, NULL, NULL);

	// g_signal_connect(data.sink  , "new-sample", G_CALLBACK(new_sample), &data);


	// data.caps = gst_caps_new_simple(
	// 	"video/x-raw",
	// 	"format"     , G_TYPE_STRING    , "format=(string)BGRx",
	// 	NULL);

	// 	data.source    = gst_element_factory_make( "rtspsrc"     , "source");
	// 	data.rtppay    = gst_element_factory_make( "rtph264depay", "depayl");
	// 	data.parse     = gst_element_factory_make( "h264parse"   , "parse" );
	// 	data.identity  = gst_element_factory_make( "identity"    , "identity" );
	// 	data.decoder   = gst_element_factory_make("nvv4l2decoder", "nvv4l2-decoder");
  	// 	data.nvvidconv =
    //   		gst_element_factory_make("nvvideoconvert", "nvvideo-converter");
  	// 	data.vidconv =
    //   		gst_element_factory_make("videoconvert", "video-converter");
  				
	// 	  data.streammux =
    //   		gst_element_factory_make("nvstreammux", "stream-muxer");
  	// 	data.nvosd =
    //   		gst_element_factory_make("nvdsosd", "nv-onscreendisplay");
  	// 	data.transform =
    //   		gst_element_factory_make("nvegltransform", "nvegl-transform");

	// 	data.capsfilter = gst_element_factory_make("capsfilter", "capsfilter");

	// 	g_object_set(data.capsfilter    , "caps"        , data.caps        , NULL);
		
	// 	// if (data.host_cpu_ == "aarch64")
	// 	// {
	// 		// data.decoder = gst_element_factory_make( "omxh264dec"  , "decode");
	// 	// }
	// 	// else
	// 	// {
	// 	// 	data.decodebin = gst_element_factory_make( "avdec_h264"  , "decode");
	// 	// }

	// 	// data.sink      = gst_element_factory_make( "nveglglessink"     , "sink"  );
	// 	data.sink      = gst_element_factory_make( "appsink"     , "sink"  );

		// set up link
		// g_object_set (G_OBJECT (data.source)   , "latency"    , 200        , NULL);
		// // g_object_set (G_OBJECT (data.decoder), "max-threads", 6           , "format" , "RGBA" ,NULL);
		// g_object_set (G_OBJECT (data.sink)     , "sync"       , TRUE       ,"drop", TRUE, NULL);
		// g_object_set(GST_OBJECT(data.source)   , "location"   , _uri.c_str(), NULL);

		// g_object_set(G_OBJECT(data.streammux), "width", MUXER_OUTPUT_WIDTH, "height",
		// 			MUXER_OUTPUT_HEIGHT, "batch-size", 1, "batched-push-timeout",
		// 			MUXER_BATCH_TIMEOUT_USEC, NULL);
		// // connect appsink to caps
		// //GstCaps* filtercaps = gst_caps_from_string("application/x-rtp");
		// //g_object_set (G_OBJECT (data.filter1), "caps", filtercaps, NULL);
		// //gst_caps_unref(filtercaps);
  		// g_object_set (data.sink, "emit-signals", TRUE, NULL);

		// gst_bin_add_many (GST_BIN (_pipeline), data.source
		// 									//  , data.identity
		//                                      , data.rtppay
		//                                      , data.parse
		// 									 , data.decoder
		// 									 , data.nvvidconv
		// 									 ,data.capsfilter
		// 									, data.vidconv
		// 									//  , data.capsfilter
		// 									//  , data.streammux
		// 									//  , data.nvvidconv
		// 									//  , data.nvosd
		// 									//  , data.transform
		// 									 , data.sink
		// 									 , NULL);




		// if(!gst_element_link_many(			  data.rtppay
		// 									, data.parse
		// 									, data.decoder
		// 									//  , data.nvvidconv

		// 									,NULL))
		// {
		//     printf("\nFailed to link parse to sink");
		// }
		// if(!gst_element_link_many(			 data.decoder
		// 											 , data.nvvidconv
		// 											 , data.capsfilter
		// 											 , data.vidconv

		// 									//  data.streammux
		// 									// , data.nvvidconv
		// 									// , data.nvosd
		// 									// , data.transform
		// 									// , data.capsfilter
		// 									, data.sink
		// 									,NULL))
		// {
		//     printf("\nFailed to link parse to sink");
		// }
		
		// // listen for newly created pads
		// g_signal_connect(data.source, "pad-added" , G_CALLBACK(cb_new_rtspsrc_pad), data.rtppay);
		// // g_signal_connect(data.identity, "handoff" , G_CALLBACK(handoff), &data);
		// g_signal_connect(data.rtppay, "pad-added" , G_CALLBACK(on_pad_added), data.parse);
		//  g_signal_connect(data.sink  , "new-sample", G_CALLBACK(new_sample), &data);


		// start playing
		ret = gst_element_set_state((GstElement*)pipeline, GST_STATE_PLAYING);


		if (ret == GST_STATE_CHANGE_FAILURE)
		{
			g_printerr("Unable to set the pipeline to the playing state.\n");
			gst_object_unref(pipeline);
		}

		if ((bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline))) != NULL)
		{
			gst_bus_enable_sync_message_emission(bus);
			g_signal_connect(bus, "sync-message", G_CALLBACK(_onBusMessage), this);
			gst_object_unref(bus);
			bus = NULL;
		}

		GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL,
								  "pipeline-paused");
		running = gst_element_set_state((GstElement*)pipeline, GST_STATE_PLAYING) !=
				  GST_STATE_CHANGE_FAILURE;
	} while (0);

	if (!running)
	{
		g_print("RtspReceiver::start() failed");

		// In newer versions, the pipeline will clean up all references that are
		// added to it
		if (pipeline != NULL)
		{
			gst_object_unref(pipeline);
			pipeline = NULL;
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
		GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline), GST_DEBUG_GRAPH_SHOW_ALL,
								  "pipeline-playing");
		_running = true;
		g_print("Running\n");
	}
	_starting = false;
}

bool RtspReceiver::getStreamAlive()
{
	bool tmp = this->data._stream_alive;
	this->data._stream_alive = false;
	return tmp;
}

void handoff(GstElement *sink, GstBuffer* buffer,CustomData *data)
{
	RESERVE(sink);

    GstBuffer *frame = NULL;
    frame = gst_buffer_ref(buffer);
    if (frame) 
	{
        GstMapInfo info;
        if(gst_buffer_map(frame, &info, GST_MAP_READ))
		{
            uint32_t timestamp = 0;
            if(info.size > 8) 
			{
                timestamp = (uint32_t)(info.data[4] << 24 | info.data[5] << 16 | info.data[6] << 8 | info.data[7]);
            }
            data->_timestamp = timestamp / 90;
            gst_buffer_unmap(frame, &info);
            gst_buffer_unref(frame);
        }
    }
}

/* The appsink has received a buffer */
void new_sample(GstAppSink *sink, CustomData *data)
{
	GstSample   *sample;
	GstBuffer   *buffer;
	GstMapInfo  map;
    gboolean    res;
	/* Retrieve the buffer */
	g_signal_emit_by_name(sink, "pull-sample", &sample);
	auto start = std::chrono::system_clock::now();
    g_print ("@@@@@\n");

	// if (sample)
	// {
	// 	// get picture format
	// 	GstCaps *caps;
	// 	GstStructure *s;
    // 	caps = gst_sample_get_caps (sample);
    // 	if (!caps) 
	// 	{
    // 	  g_print ("could not get snapshot format\n");
    // 	  exit (-1);
    // 	}
    // 	s = gst_caps_get_structure (caps, 0);
    // 	/* we need to get the final caps on the buffer to get the size */
    // 	res = gst_structure_get_int (s, "width", &data->_width);
    // 	res |= gst_structure_get_int (s, "height", &data->_height);

	// 	if (!res) 
	// 	{
	// 		g_print ("could not get snapshot dimension\n");
	// 		exit (-1);
	// 	}

	// 	// get buffer
	// 	buffer = gst_sample_get_buffer(sample);
	// 	gst_buffer_map(buffer, &map, GST_MAP_READ);

	// 	cv::Mat mRGB;
	// 	auto msg = std::make_shared<sensor_msgs::msg::Image>();

	// 	bool use_rgb = true;
	// 	if (use_rgb)
	// 	{
	// 		cv::Mat t = cv::Mat(data->_height + data->_height / 2 , data->_width,
	// 							CV_8UC1, (void *)map.data);

	// 		if (data->host_cpu_ == "aarch64")
	// 		{
	// 			cvtColor(t, mRGB, CV_YUV2BGR_NV12);
	// 		}
	// 		else
	// 		{
	// 			cvtColor(t, mRGB, CV_YUV420p2RGB);
	// 		}
	// 		convert_frame_to_message(mRGB, 10, msg);
	// 	}
	// 	else
	// 	{

	// 		cv::Mat t = cv::Mat(data->_height /*+ data->_height / 2*/ , data->_width,
	// 							CV_8UC1, (void *)map.data);
	// 		convert_frame_to_message(t, 10, msg);
	// 		// cv::imshow("usb", t);
	// 	}
	// 	if (use_rgb && data->_image_display)
	// 	{
	// 		cv::imshow("usb", mRGB);
	// 		cv::waitKey(1);
	// 	}

	// 	int secs = data->_timestamp / 1000;

	// 	// TODO: add frame_id
	// 	msg->header.stamp.sec = secs;
	// 	msg->header.stamp.nanosec = (data->_timestamp % 1000) * 1000000;
		
	// 	if (data->_verbose)
	// 	{
	// 		std::cout << secs << " -- " << data->_timestamp << std::endl;
	// 	}
		
	// 	try 
	// 	{
	// 		data->_stream_alive = true;
	// 		data->image_pub_->publish(*msg);
	// 	} catch (std::exception & ex) {}

	// 	gst_buffer_unmap(buffer, &map);
	// 	gst_sample_unref(sample);
	// }
	auto end = std::chrono::system_clock::now();
	// std::chrono::duration<std::chrono::milliseconds> diff = end - start;
	// std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(end-start).count() << std::endl;
}

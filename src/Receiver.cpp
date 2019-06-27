#include "Receiver.hpp"

Receiver::Receiver()
    : _running(false),
      _starting(false),
      _stopping(false),
      _tee(nullptr),
      _pipeline(nullptr),
      _videoSink(nullptr),
      _uri("") {}

Receiver::~Receiver() {}

void Receiver::start() {
  if (_uri.empty()) {
    g_print("Receiver::start() failed because URI is not specified");
    return;
  }
  if (_running) {
    g_print("Already running!");
    return;
  }
}

void Receiver::stop() {
  g_print("stop()");
  if (!_streaming) {
    _shutdownPipeline();
  } else if (_pipeline != NULL && !_stopping) {
    g_print("Stopping _pipeline");
    gst_element_send_event(_pipeline, gst_event_new_eos());
    _stopping = true;
    GstBus* bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline));
    GstMessage* message = gst_bus_timed_pop_filtered(
        bus, GST_CLOCK_TIME_NONE,
        (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
    gst_object_unref(bus);
    if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR) {
      _shutdownPipeline();
      g_print("Error stopping pipeline!");
    } else if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_EOS) {
      _handleEOS();
    }
    gst_message_unref(message);
  }
}

void Receiver::_shutdownPipeline() {
  if (!_pipeline) {
    g_print("No pipeline");
    return;
  }
  GstBus* bus = NULL;
  if ((bus = gst_pipeline_get_bus(GST_PIPELINE(_pipeline))) != NULL) {
    gst_bus_disable_sync_message_emission(bus);
    gst_object_unref(bus);
    bus = NULL;
  }
  gst_element_set_state(_pipeline, GST_STATE_NULL);
  gst_bin_remove(GST_BIN(_pipeline), _videoSink);
  gst_object_unref(_pipeline);
  _streaming = false;
  _stopping = false;
  _running = false;
}

void Receiver::_handleEOS() {
  if (_stopping) {
    _shutdownPipeline();
    g_print("Stopped");
  }
}

void Receiver::setUri(const std::string uri) { _uri = uri; }

void Receiver::setDisplay(bool display) { _image_display = display; }

void Receiver::setVerbose(bool verbose) { _verbose = verbose; }

void Receiver::setStreamAlive(bool streamalive) { _stream_alive = streamalive; }

bool Receiver::getStreamAlive() { return _stream_alive; }

gboolean Receiver::_onBusMessage(GstBus* bus, GstMessage* msg, gpointer data) {
  assert(msg != NULL && data != NULL);
  Receiver* pThis = (Receiver*)data;
  RESERVE(pThis);
  RESERVE(bus);

  switch (GST_MESSAGE_TYPE(msg)) {
    case (GST_MESSAGE_ERROR): {
      gchar* debug;
      GError* error;
      gst_message_parse_error(msg, &error, &debug);
      g_free(debug);
      g_print("%s", error->message);
      g_error_free(error);
    } break;
    case (GST_MESSAGE_EOS):
      break;
    case (GST_MESSAGE_STATE_CHANGED):
      break;
    default:
      break;
  }

  return TRUE;
}

GstBus* Receiver::getBus() { return gst_element_get_bus(_pipeline); }

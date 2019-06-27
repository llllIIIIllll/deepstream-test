#ifndef __RECEIVER__HPP__
#define __RECEIVER__HPP__

#include <assert.h>
#include <gst/gst.h>
#include <string.h>
#include <iostream>
#include "common.hpp"
//#include "MyTimer.hpp"

class Receiver {
 public:
  explicit Receiver();
  ~Receiver();

  virtual void start();
  virtual bool getStreamAlive();
  
  void stop();
  void setUri(const std::string uri);
  void setDisplay(bool display);
  void setVerbose(bool verbose);
  void setStreamAlive(bool streamalive);
  GstBus *getBus();

  void _handleEOS();
  void _shutdownPipeline();
  bool _running;
  bool _streaming;
  bool _starting;
  bool _stopping;
  GstElement *_tee;
  // MyTimer         _myTimer;

  GstElement *_pipeline;
  GstElement *_videoSink;

  std::string _uri;
  bool        _image_display;
  bool        _verbose;
  bool        _stream_alive;

  static gboolean _onBusMessage(GstBus *bus, GstMessage *message,
                                gpointer user_data);
};

#endif  //__RECEIVER__HPP__

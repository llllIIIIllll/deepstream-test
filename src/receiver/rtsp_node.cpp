// Copyright 2016 Open Source Robotics Foundation, Inc.                      
//                                                                           
// Licensed under the Apache License, Version 2.0 (the "License");           
// you may not use this file except in compliance with the License.          
// You may obtain a copy of the License at                                   
//                                                                           
//     http://www.apache.org/licenses/LICENSE-2.0                            
//                                                                           
// Unless required by applicable law or agreed to in writing, software       
// distributed under the License is distributed on an "AS IS" BASIS,         
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  
// See the License for the specific language governing permissions and       
// limitations under the License.                                            

#include <memory>                                                            
#include "rclcpp/rclcpp.hpp"                                                 

#include "rtsp_receiver_component.h"
//#include "rtsp_receiver/RtspReceiver.hpp"

#include <thread>
#include <chrono>

using namespace std::chrono_literals;

//TODO: Tabular
int main(int argc, char * argv[])                                            
{                                                                            
   	gst_init (&argc, &argv);  
	rclcpp::init(argc, argv);                                                  

   	rclcpp::spin(std::make_shared<ros2_videostreamer::RtspReceiverNode>());
   	rclcpp::shutdown();
   return 0;                                                                  
}                                                                            


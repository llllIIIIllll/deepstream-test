#include "rtsp_receiver_component.h"
// #include "rclcpp_action/rclcpp_action.hpp"

#define NODE_NAME "rtsp"
#define NAME_SPACE ""
namespace ros2_videostreamer
{
	RtspReceiverNode::RtspReceiverNode()
		:Node(NODE_NAME,
        NAME_SPACE,
        rclcpp::NodeOptions(
            rclcpp::NodeOptions()
            .allow_undeclared_parameters(true)
            .automatically_declare_parameters_from_overrides(true)
            )
        )
	{
        using namespace std::placeholders;
        
        this->receiver_  = std::make_shared<RtspReceiver>();
        
		param_switch_service_tracker_name_ = "/tld_kcf_tracker/switch_on";
		param_switch_service_detector_name_ = "/detector/switch";
        param_rtsp_uri_ = "rtsp://192.168.1.21:554/ch4";
        param_rtsp_uri_topic_ = "rtsp_uri";

		this->stream_restart_ = false;
        this->param_image_display_ = true;
        this->param_verbose_ = true;
        this->param_auto_start_ = true;

        this->get_parameter_or("param_rtsp_uri", param_rtsp_uri_,param_rtsp_uri_);
        this->get_parameter_or("param_rtsp_uri_topic", param_rtsp_uri_topic_,param_rtsp_uri_topic_);
        this->get_parameter_or("display", param_image_display_, param_image_display_);
        this->get_parameter_or("verbose", param_verbose_, param_verbose_);
        this->get_parameter_or("auto_start", param_auto_start_,param_auto_start_);
        
        this->switch_on_ = this->param_auto_start_;
        this->turn_on_or_off_ = this->param_auto_start_;

        auto image_pub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        auto image_sub_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        // auto switch_qos_profile = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();

        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/image_raw", image_pub_qos_profile);

        // // tracker switch
        // auto switch_tracker_cb = std::bind(&RtspReceiverNode::switch_service_tracker_callback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
        // switch_service_tracker_ = this->create_service<std_srvs::srv::SetBool>(
        //     param_switch_service_tracker_name_, switch_tracker_cb);
            
        // // detector switch
        // auto switch_detector_cb = std::bind(&RtspReceiverNode::switch_service_detector_callback, this, std::placeholders::_1, std::placeholders::_2,std::placeholders::_3);
        // switch_service_detector_ = this->create_service<std_srvs::srv::SetBool>(
        //     param_switch_service_detector_name_, switch_detector_cb);
            
        auto rtsp_cb = std::bind(&RtspReceiverNode::topic_rtsp_uri_callback_shared, this, std::placeholders::_1);
        rtsp_uri_ = this->create_subscription<std_msgs::msg::String>(
            param_rtsp_uri_topic_, image_sub_qos_profile, rtsp_cb);

        this->timer_check_alive_ = this->create_wall_timer(
            std::chrono::seconds(this->timer_check_alive_callback_interval), 
            std::bind(&RtspReceiverNode::timer_check_alive_callback, this));
        // this->timer_check_alive_->cancel();


        // this->action_server_ = rclcpp_action::create_server<Switch>(
        //     this->get_node_base_interface(),
        //     this->get_node_clock_interface(),
        //     this->get_node_logging_interface(),
        //     this->get_node_waitables_interface(),
        //     "switch",
        //     std::bind(&RtspReceiverNode::handle_goal, this, _1, _2),
        //     std::bind(&RtspReceiverNode::handle_cancel, this, _1),
        //     std::bind(&RtspReceiverNode::handle_accepted, this, _1)
        // );

        if (this->param_auto_start_)
        {
            this->start_stream();
        }
	}

    // void RtspReceiverNode::switch_service_tracker_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    // {
    //     // RCLCPP_INFO(this->get_logger(), "in switch CB");
    //     static bool recount = true;
    //     if (request->data)
    //     {
    //         recount = true;
    //         RCLCPP_INFO(this->get_logger(), "switch on");
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "switch off");
    //     }
        
    //     // if no switch on message coming, turn off start time will remain at the first switch off message's time point
    //     if(switch_on_ && !request->data && recount == true)
    //     {
    //         this->turn_off_count_start_ = std::chrono::system_clock::now();
    //         recount = false;
    //     }
        
    //     this->tracker_turn_on_or_off_ = request->data;
        
    //     RESERVE(request_header);
    //     RESERVE(request);
    //     RESERVE(response);
    // }

    //     void RtspReceiverNode::switch_service_detector_callback(const std::shared_ptr<rmw_request_id_t> request_header,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    //     const std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    // {
    //     // RCLCPP_INFO(this->get_logger(), "in switch CB");
    //     static bool recount = true;
    //     if (request->data)
    //     {
    //         recount = true;
    //         RCLCPP_INFO(this->get_logger(), "switch on");
    //     }
    //     else
    //     {
    //         RCLCPP_INFO(this->get_logger(), "switch off");
    //     }
        
    //     // if no switch on message coming, turn off start time will remain at the first switch off message's time point
    //     if(switch_on_ && !request->data && recount == true)
    //     {
    //         this->turn_off_count_start_ = std::chrono::system_clock::now();
    //         recount = false;
    //     }
        
    //     this->detector_turn_on_or_off_ = request->data;
        
    //     RESERVE(request_header);
    //     RESERVE(request);
    //     RESERVE(response);
    // }

    void RtspReceiverNode::topic_rtsp_uri_callback_shared(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "stream running : %s", msg->data);
        if (this->param_rtsp_uri_ != msg->data)
        {
            this->param_rtsp_uri_ = msg->data;
            if (switch_on_)
            {
                this->stop_stream();
                this->start_stream();
            }
        }

    }

	RtspReceiverNode::~RtspReceiverNode()
	{}

    void RtspReceiverNode::start_stream()
    {        
        this->receiver_->data.image_pub_ = image_pub_;
        this->receiver_->setDisplay(param_image_display_);
        this->receiver_->setVerbose(param_verbose_);
        this->receiver_->setUri(param_rtsp_uri_);
        this->receiver_->setStreamAlive(false);
        
        this->receiver_->start();
    }
    
    void RtspReceiverNode::stop_stream()
    {
        if (this->receiver_->_running && !this->receiver_->_stopping)
            this->receiver_->stop();
    }

    void RtspReceiverNode::wait_count_subscribers()
    {
        while (true)
        {
            static int cur_sub_counts = count_subscribers("/image_raw");
            auto event = this->get_graph_event();
            this->wait_for_graph_change(event, std::chrono::nanoseconds(10000000000));
            {
                int tmp = count_subscribers("/image_raw");
                if (tmp != cur_sub_counts)
                {
                    cur_sub_counts = tmp;
                    std::cout << "sub counts change!" << std::endl;
                    this->timer_check_alive_callback();
                }
            }
        }
    }

    void RtspReceiverNode::timer_check_alive_callback()
    {
        // this remain for action
        static int restart_count = 0;
        auto turn_off_count_end = std::chrono::system_clock::now();
        
        // this is for sub count
        size_t sub_counts = this->count_subscribers("/image_raw");
        std::cout << "sub counts: " << sub_counts << std::endl;
        static int sub_counts_turn_off = 0; 
        
        if (sub_counts != 0)
        {
            this->turn_on_or_off_ = true;
            sub_counts_turn_off = 0;
        }
        else
        {
            this->turn_on_or_off_ = false;
            sub_counts_turn_off++;
        }

        // only both off turn_on_or_off_ will be off
        // this->turn_on_or_off_ = this->tracker_turn_on_or_off_ || this->detector_turn_on_or_off_;
        
        // check turn on or on, if turn off triggered, it will be off in 5 minutes;
        if (this->turn_on_or_off_)
        {
            this->switch_on_ = true;
        }
        else if (std::chrono::duration_cast<std::chrono::duration<int>>
                    (turn_off_count_end - this->turn_off_count_start_).count() > 10
                || (sub_counts_turn_off >= 5)
              )
        {
            this->switch_on_ = false;
            sub_counts_turn_off = 0;
        }
        
        if (!this->switch_on_)
        {
            this->stream_alive_ = false;
            this->stop_stream();
            return;
        }
        
        // switch is on, try to get stream
        this->stream_alive_ = this->receiver_->getStreamAlive();
        
        if (!this->stream_alive_ && restart_count >= 3)
        {
            this->stop_stream();
            restart_count = 0;
        } else if (!this->stream_alive_)
        {
            RCLCPP_INFO(this->get_logger(), "try to get stream video");
            restart_count++;
            this->start_stream();
        }
        else 
        {
            this->receiver_->setStreamAlive(false);
        }
    }
    
    // rclcpp_action::GoalResponse RtspReceiverNode::handle_goal (const rclcpp_action::GoalUUID & uuid,
    //     std::shared_ptr<const Switch::Goal> goal)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received goal request with switch on: %d", goal->switch_on);
    //     (void)uuid;
    //     // reject
    //     if (goal->switch_on == this->receiver_->getStreamAlive()) 
    //     {
    //         return rclcpp_action::GoalResponse::REJECT;
    //     }
    //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    // }
    
    // void RtspReceiverNode::execute(const std::shared_ptr<GoalHandleSwitch> goal_handle)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Executing goal");
    //     rclcpp::Rate loop_rate(1);
    //     const auto goal = goal_handle->get_goal();
    //     auto feedback = std::make_shared<Switch::Feedback>();
    //     auto & stream_running = feedback->stream_running;
    //     auto & image_publish = feedback->image_publish;
    //     auto result = std::make_shared<Switch::Result>();
        
    //     this->turn_on_or_off_ = goal->switch_on;
    //     this->turn_off_count_start_ = std::chrono::system_clock::now();

    //     for (int i = 0; i < 100; ++i)
    //     {
    //         if (goal_handle->is_canceling()) 
    //         {
    //             this->turn_on_or_off_  =  !goal->switch_on;
    //             result->switch_state = this->receiver_->getStreamAlive();
    //             goal_handle->canceled(result);
    //             RCLCPP_INFO(this->get_logger(), "Goal Canceled");
    //             return;
    //         }
    //         stream_running = this->receiver_->_running;
    //         image_publish = this->receiver_->getStreamAlive();
            
    //         goal_handle->publish_feedback(feedback);
    //         RCLCPP_INFO(this->get_logger(), "Publish Feedback");
            
    //         if (goal->switch_on && image_publish)
    //         {
    //             // turn on success
    //             break;
    //         }
    //         else if (!goal->switch_on && !stream_running)
    //         {
    //             // turn off success
    //             break;
    //         }
    //         loop_rate.sleep();
    //     }
        
    //     // Check if goal is done
    //     if (rclcpp::ok()) 
    //     {
    //         result->switch_state = this->stream_alive_;
    //         goal_handle->succeed(result);
    //         RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
    //     }
    // }
          
    // void RtspReceiverNode::handle_accepted(const std::shared_ptr<GoalHandleSwitch> goal_handle)
    // {
    //     using namespace std::placeholders;
    //     // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    //     std::thread{std::bind(&RtspReceiverNode::execute, this, _1), goal_handle}.detach();
    // }
    
    // rclcpp_action::CancelResponse RtspReceiverNode::handle_cancel(const std::shared_ptr<GoalHandleSwitch> goal_handle)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    //     (void)goal_handle;
    //     return rclcpp_action::CancelResponse::ACCEPT;
    // }

    
};

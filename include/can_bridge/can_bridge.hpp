#ifndef CAN_BRIDGE_HPP
#define CAN_BRIDGE_HPP

#include <rclcpp/rclcpp.hpp>
#include "linux/can.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <cstdio>
#include <memory>
#include <string>
#include <errno.h>
#include <thread>
#include <sstream>
#include <boost/process.hpp>

// #include "lart_msgs/msg/state.hpp"
// #include "lart_msgs/msg/mission.hpp"
// #include "lart_msgs/msg/as_status.hpp"
// #include "lart_msgs/msg/dynamics_cmd.hpp"
// #include "lart_msgs/msg/dynamics.hpp"
// #include "lart_msgs/msg/slam_stats.hpp"

#include "lart_msgs/msg/dv_dynamics1.hpp"
#include "lart_msgs/msg/dv_dynamics2.hpp"
#include "lart_msgs/msg/dv_status.hpp"
#include "lart_msgs/msg/jetson.hpp"
#include "lart_msgs/msg/res.hpp"
#include "lart_msgs/msg/vcu_hv.hpp"
#include "lart_msgs/msg/vcu_ign_r2d.hpp"
#include "lart_msgs/msg/vcu_rpm.hpp"
#include "lart_msgs/msg/vcu_rpm_target.hpp"
#include "lart_msgs/msg/vcu_torque_target.hpp"
#include "lart_msgs/msg/acu.hpp"
#include "lart_msgs/msg/aqt1.hpp"
#include "lart_msgs/msg/aqt2.hpp"
#include "lart_msgs/msg/aqt3.hpp"
#include "lart_msgs/msg/aqt4.hpp"
#include "lart_msgs/msg/aqt7.hpp"
#include "lart_msgs/msg/asf_signals.hpp"
#include "lart_msgs/msg/cubemars_feedback.hpp"
#include "lart_msgs/msg/cubemars_possition_loop.hpp"

#include "T26_DBC/generated/autonomous_t26/autonomous_t26.h"

#define CAN_INTERFACE "can0"

namespace bp = boost::process;

class CanBridge : public rclcpp::Node
{
    public:
        CanBridge();

    private:
        int s; //socket descriptor
        std::mutex socket_mutex;

        bool nodes_initialized = false;
        
        
        void read_can_frame();
        void send_can_frame(struct can_frame frame);
        void send_can_frames();
        void handle_can_frame(struct can_frame frame);
        

        // // Publishers
        // rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_pub;
        // rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub;
        // rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr dynamics_pub;

        // // Subscribers
        // rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;
        

};
#endif // CAN_BRIDGE_HPP





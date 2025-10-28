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

#include "lart_msgs/msg/state.hpp"
#include "lart_msgs/msg/mission.hpp"
#include "lart_msgs/msg/as_status.hpp"
#include "lart_msgs/msg/dynamics_cmd.hpp"
#include "lart_msgs/msg/dynamics.hpp"
#include "lart_msgs/msg/slam_stats.hpp"
#include "lart_msgs/msg/vcu_hv.hpp"
#include "lart_msgs/msg/vcu_rpm.hpp"
//verificar estes 
#include "lart_msgs/msg/dyn_front_sig1.hpp"
#include "lart_msgs/msg/dyn_front_sig2.hpp"
#include "lart_msgs/msg/"
//Include example
#include "lart_msgs/msg/acu_ms.hpp"

#include "CAN_DBC/generated/Autonomous_temporary/autonomous_temporary.h"

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
        
        
        
    

        // Publishers
        rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_pub;
        rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub;
        rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr dynamics_pub;
        rclcpp::publisher<lart_msgs::msg::VcuRpm>::SharedPtr vcu_rpm_pub;
        rclcpp::Publisher<lart_msgs::msg::VcuHv>::SharedPtr vcu_hv_pub;
        rclcpp::Publisher<lart_msgs::msg::DynFrontSig1>::SharedPtr dyn_front_sig1_pub;
        rclcpp::Publisher<lart_msgs::msg::DynFrontSig2>::SharedPtr dyn_front_sig2_pub;
        

        // Subscribers
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;
        

};
#endif // CAN_BRIDGE_HPP
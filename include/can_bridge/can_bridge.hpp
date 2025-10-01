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

        void read_can_frame();
        void send_can_frame(struct can_frame frame);
        void send_can_frames();
        void handle_can_frame(struct can_frame frame);
        
        
        
        bool nodes_initialized = false;
   
};

#endif // CAN_BRIDGE_HPP
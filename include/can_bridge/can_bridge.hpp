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
#include "lart_msgs/msg/dyn_rear_sig1.hpp"
#include "lart_msgs/msg/dyn_rear_sig2.hpp"
#include "lart_msgs/msg/asf_signals.hpp"
#include "lart_msgs/msg/vcu_ign_r2d.hpp" //verificar este
#include "lart_msgs/msg/acu_status.hpp"
#include "lart_msgs/msg/maxon_status_tx.hpp"
#include "lart_msgs/msg/maxon_status2_tx.hpp"
#include "lart_msgs/msg/maxon_position_tx.hpp"
#include "lart_msgs/msg/maxon_velocity_tx.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

//Include example
#include "lart_msgs/msg/acu_ms.hpp"

#include "CAN_DBC/generated/Autonomous_temporary/autonomous_temporary.h"
#include "maxon.hpp"
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
        
        //Callbacks
        void StateCallBack(const lart_msgs::msg::State::SharedPtr msg);
        void ekfStateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        //include is on state_controler.hpp
        void ekfStatsCallback(const lart_msgs::msg::SlamStats::SharedPtr msg);

    

        // Publishers
        rclcpp::Publisher<lart_msgs::msg::State>::SharedPtr state_pub;
        rclcpp::Publisher<lart_msgs::msg::Mission>::SharedPtr mission_pub;
        rclcpp::Publisher<lart_msgs::msg::Dynamics>::SharedPtr dynamics_pub;
        rclcpp::publisher<lart_msgs::msg::VcuRpm>::SharedPtr vcu_rpm_pub;
        rclcpp::Publisher<lart_msgs::msg::VcuHv>::SharedPtr vcu_hv_pub;
        rclcpp::Publisher<lart_msgs::msg::DynFrontSig1>::SharedPtr dyn_front_sig1_pub;
        rclcpp::Publisher<lart_msgs::msg::DynFrontSig2>::SharedPtr dyn_front_sig2_pub;
        rclcpp::Publisher<lart_msgs::msg::DynRearSig1>::SharedPtr dyn_rear_sig1_pub;
        rclcpp::Publisher<lart_msgs::msg::DynRearSig2>::SharedPtr dyn_rear_sig2_pub;
        rclcpp::Publisher<lart_msgs::msg::AsfSignals>::SharedPtr asf_signals_pub;
        rclcpp::Publisher<lart_msgs::msg::VcuIgnR2d>::SharedPtr vcu_ign_r2_d_pub;
        rclcpp::Publisher<lart_msgs::msg::AcuStatus>::SharedPtr acu_status_pub;
        rclcpp::Publisher<lart_msgs::msg::MaxonStatusTx>::SharedPtr maxon_status_tx_pub;
        rclcpp::Publisher<lart_msgs::msg::MaxonStatus2Tx>::SharedPtr maxon_status2_tx_pub;
        rclcpp::Publisher<lart_msgs::msg::MaxonPositionTx>::SharedPtr maxon_position_tx_pub;
        rclcpp::Publisher<lart_msgs::msg::MaxonVelocityTx>::SharedPtr maxon_velocity_tx_pub;


        //MaxonVelocityTx
        // Subscribers
        rclcpp::Subscription<lart_msgs::msg::State>::SharedPtr state_sub;
        rlcpp::Subscription<lart_msgs::msg::PoseStamped>::SharedPtr ekf_state_sub; //verificar
        rlcpp::Subscription<lart_msgs::msg::SlamStats>::SharedPtr ekf_stats_sub;
};
#endif // CAN_BRIDGE_HPP
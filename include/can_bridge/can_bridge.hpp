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
#include <net/if.h>
#include <sys/ioctl.h>

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
        

        // Publishers

            //aquisitions
        rclcpp::Publisher<lart_msgs::msg::Aqt1>::SharedPtr aquisition1_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt2>::SharedPtr aquisition2_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt3>::SharedPtr aquisition3_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt4>::SharedPtr aquisition4_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt7>::SharedPtr aquisition7_pub_;

            //VCU
        rclcpp::Publisher<lart_msgs::msg::VcuHv>::SharedPtr vcu_hv_pub_;
        rclcpp::Publisher<lart_msgs::msg::VcuIgnR2d>::SharedPtr vcu_ign_r2d_pub_;
        rclcpp::Publisher<lart_msgs::msg::VcuRpm>::SharedPtr vcu_rpm_pub_;

            //ACU
        rclcpp::Publisher<lart_msgs::msg::Acu>::SharedPtr acu_pub_;

            //Cubemars
        rclcpp::Publisher<lart_msgs::msg::CubemarsFeedback>::SharedPtr cube_mars_feedback_pub_;

            // RES
        rclcpp::Publisher<lart_msgs::msg::Res>::SharedPtr res_pub_;


        // Subscribers

            //VCU
        rclcpp::Subscription<lart_msgs::msg::VcuRpmTarget>::SharedPtr vcu_rpm_target_sub_;
        rclcpp::Subscription<lart_msgs::msg::VcuTorqueTarget>::SharedPtr vcu_torque_target_sub_;

            //Cubemars
        rclcpp::Subscription<lart_msgs::msg::CubemarsPossitionLoop>::SharedPtr cubemars_possition_loop_sub_;

            // Handbook signals
        rclcpp::Subscription<lart_msgs::msg::AsfSignals>::SharedPtr asf_signals_sub_;
        rclcpp::Subscription<lart_msgs::msg::DvDynamics1>::SharedPtr dv_dynamics1_sub_;
        rclcpp::Subscription<lart_msgs::msg::DvDynamics2>::SharedPtr dv_dynamics2_sub_;
        rclcpp::Subscription<lart_msgs::msg::DvStatus>::SharedPtr dv_status_sub_;

            // Jetson
        rclcpp::Subscription<lart_msgs::msg::Jetson>::SharedPtr jetson_sub_;

        // Handlers for subscribers
        void handle_vcu_torque_target_message(const lart_msgs::msg::VcuTorqueTarget::SharedPtr msg);
        void handle_vcu_rpm_target_message(const lart_msgs::msg::VcuRpmTarget::SharedPtr msg);
        
        void handle_cubemars_possition_loop_message(const lart_msgs::msg::CubemarsPossitionLoop::SharedPtr msg);

        void handle_asf_signals_message(const lart_msgs::msg::AsfSignals::SharedPtr msg);
        void handle_dv_dynamics1_message(const lart_msgs::msg::DvDynamics1::SharedPtr msg);
        void handle_dv_dynamics2_message(const lart_msgs::msg::DvDynamics2::SharedPtr msg);
        void handle_dv_status_message(const lart_msgs::msg::DvStatus::SharedPtr msg);
        
        void handle_jetson_message(const lart_msgs::msg::Jetson::SharedPtr msg);

};
#endif // CAN_BRIDGE_HPP





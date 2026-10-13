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
#include <chrono>
#include <algorithm>
#include <net/if.h>
#include <sys/ioctl.h>
#include "topics.h"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "xsens_mti_ros2_driver/msg/xs_status_word.hpp"

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
#include "lart_msgs/msg/aqt4.hpp"
#include "lart_msgs/msg/aqt7.hpp"
#include "lart_msgs/msg/asf_signals.hpp"
#include "lart_msgs/msg/cubemars_feedback.hpp"
#include "lart_msgs/msg/cubemars_position_loop.hpp"

#include "T26_DBC/generated/autonomous_t26/autonomous_t26.h"

#define CAN_INTERFACE "can0"
#define MAXON_HEARTBEAT_ID 0x708

// Xsens driver (xsens_mti_ros2_driver) topics forwarded to the IMU_* frames
#define XSENS_ACCELERATION_TOPIC "/imu/acceleration"
#define XSENS_ANGULAR_VELOCITY_TOPIC "/imu/angular_velocity"
#define XSENS_GNSS_POSE_TOPIC "/gnss_pose"
#define XSENS_ODOMETRY_TOPIC "/odometry"
#define XSENS_EULER_TOPIC "/filter/euler"
#define XSENS_STATUS_TOPIC "/status"
#define XSENS_TEMPERATURE_TOPIC "/temperature"
#define XSENS_FILTER_VELOCITY_TOPIC "/filter/velocity"
#define XSENS_FREE_ACCELERATION_TOPIC "/filter/free_acceleration"

// Max rate (Hz) of each IMU_* frame on the bus, faster samples are dropped
#define IMU_CAN_FREQUENCY 100


class CanBridge : public rclcpp::Node
{
    public:
        CanBridge();

    private:
        int s; //socket descriptor
        std::mutex socket_mutex;

        bool asms = false;
        
        void read_can_frame();
        void send_can_frame(struct can_frame frame); // only while ASMS is on
        void write_can_frame(struct can_frame frame); // always
        void send_can_frames();
        void handle_can_frame(struct can_frame frame);

        bool imu_frame_due(std::chrono::steady_clock::time_point &next_send);

        void service_bag_start();
        void start_recording_request();
        void stop_recording_request();
        void handle_start_recording_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);
        void hanlde_stop_recording_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future);


        // Publishers
        
            //aquisitions
        rclcpp::Publisher<lart_msgs::msg::Aqt1>::SharedPtr aquisition1_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt4>::SharedPtr aquisition4_pub_;
        rclcpp::Publisher<lart_msgs::msg::Aqt7>::SharedPtr aquisition7_pub_;

            //VCU
        rclcpp::Publisher<lart_msgs::msg::VcuHv>::SharedPtr vcu_hv_pub_;
        rclcpp::Publisher<lart_msgs::msg::VcuIgnR2d>::SharedPtr vcu_ign_r2d_pub_;
        rclcpp::Publisher<lart_msgs::msg::VcuRpm>::SharedPtr vcu_rpm_pub_;

            //ACU
        rclcpp::Publisher<lart_msgs::msg::Acu>::SharedPtr acu_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr acu_mission_pub_;


            //Cubemars
        rclcpp::Publisher<lart_msgs::msg::CubemarsFeedback>::SharedPtr cube_mars_feedback_pub_;

            // RES
        rclcpp::Publisher<lart_msgs::msg::Res>::SharedPtr res_pub_;

            // Handbook signals
        rclcpp::Publisher<lart_msgs::msg::AsfSignals>::SharedPtr asf_signals_pub_;

        // Subscribers

            //VCU
        rclcpp::Subscription<lart_msgs::msg::VcuRpmTarget>::SharedPtr vcu_rpm_target_sub_;
        rclcpp::Subscription<lart_msgs::msg::VcuTorqueTarget>::SharedPtr vcu_torque_target_sub_;

            //Cubemars
        rclcpp::Subscription<lart_msgs::msg::CubemarsPositionLoop>::SharedPtr cubemars_position_loop_sub_;

            // Handbook signals
        rclcpp::Subscription<lart_msgs::msg::DvDynamics1>::SharedPtr dv_dynamics1_sub_;
        rclcpp::Subscription<lart_msgs::msg::DvDynamics2>::SharedPtr dv_dynamics2_sub_;
        rclcpp::Subscription<lart_msgs::msg::DvStatus>::SharedPtr dv_status_sub_;

            // Jetson
        rclcpp::Subscription<lart_msgs::msg::Jetson>::SharedPtr jetson_sub_;

            // IMU
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_acceleration_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_angular_velocity_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr imu_gnss_pose_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr imu_odometry_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_euler_sub_;
        rclcpp::Subscription<xsens_mti_ros2_driver::msg::XsStatusWord>::SharedPtr imu_status_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr imu_temperature_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_filter_velocity_sub_;
        rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_free_acceleration_sub_;

        // Next time each IMU topic may be sent, see IMU_CAN_FREQUENCY
        std::chrono::steady_clock::time_point imu_acceleration_next_send;
        std::chrono::steady_clock::time_point imu_angular_velocity_next_send;
        std::chrono::steady_clock::time_point imu_gnss_pose_next_send;
        std::chrono::steady_clock::time_point imu_odometry_next_send;
        std::chrono::steady_clock::time_point imu_euler_next_send;
        std::chrono::steady_clock::time_point imu_status_next_send;
        std::chrono::steady_clock::time_point imu_temperature_next_send;
        std::chrono::steady_clock::time_point imu_filter_velocity_next_send;
        std::chrono::steady_clock::time_point imu_free_acceleration_next_send;

        // Handlers for subscribers
        void handle_vcu_torque_target_message(const lart_msgs::msg::VcuTorqueTarget::SharedPtr msg);
        void handle_vcu_rpm_target_message(const lart_msgs::msg::VcuRpmTarget::SharedPtr msg);
        
        void handle_cubemars_position_loop_message(const lart_msgs::msg::CubemarsPositionLoop::SharedPtr msg);

        void handle_dv_dynamics1_message(const lart_msgs::msg::DvDynamics1::SharedPtr msg);
        void handle_dv_dynamics2_message(const lart_msgs::msg::DvDynamics2::SharedPtr msg);
        void handle_dv_status_message(const lart_msgs::msg::DvStatus::SharedPtr msg);
        
        void handle_jetson_message(const lart_msgs::msg::Jetson::SharedPtr msg);

        void handle_imu_acceleration_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void handle_imu_angular_velocity_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void handle_imu_gnss_pose_message(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void handle_imu_odometry_message(const nav_msgs::msg::Odometry::SharedPtr msg);
        void handle_imu_euler_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void handle_imu_status_message(const xsens_mti_ros2_driver::msg::XsStatusWord::SharedPtr msg);
        void handle_imu_temperature_message(const sensor_msgs::msg::Temperature::SharedPtr msg);
        void handle_imu_filter_velocity_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);
        void handle_imu_free_acceleration_message(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg);

};
#endif // CAN_BRIDGE_HPP





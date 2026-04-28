#include "can_bridge/can_bridge.hpp"

CanBridge::CanBridge() : Node("can_bridge"){
  RCLCPP_INFO(this->get_logger(), "Can Bridge Node has been started");

  // create a socket
	if((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
		return;
	}

  //define can interface
  struct ifreq ifr;
  strcpy(ifr.ifr_name, CAN_INTERFACE);
  ioctl(this->s, SIOCGIFINDEX, &ifr);
  
  // bind the socket to the CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if(bind(this->s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));
      exit(1);
  }

  // //Initiate publishers
  // this->state_pub = this->create_publisher<lart_msgs::msg::State>("/state/acu", 10);
  // this->mission_pub = this->create_publisher<lart_msgs::msg::Mission>("/mission/acu", 10);
  // this->dynamics_pub = this->create_publisher<lart_msgs::msg::Dynamics>("/dynamics", 10);


  // create a thread to read CAN frames
  std::thread read_can_thread(&CanBridge::read_can_frame, this);
  read_can_thread.detach();

  std::thread send_can_thread(&CanBridge::send_can_frames, this);
  send_can_thread.detach();
}

void CanBridge::read_can_frame(){
  while(rclcpp::ok()) {
		struct can_frame frame;
		int nbytes = read(this->s, &frame, sizeof(frame));
		if(nbytes < 0) {
			RCLCPP_ERROR(this->get_logger(), "Failed to read CAN frame: %s", strerror(errno));
			return;
		}
		handle_can_frame(frame);// Send the received can frame to a function that handles it
		//RCLCPP_INFO(this->get_logger(), "RECEIVED A CAN FRAME");
	}
}

void CanBridge::send_can_frame(struct can_frame frame){
  std::lock_guard<std::mutex> guard(this->socket_mutex);
  if(write(this->s, &frame, sizeof(frame)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
  }
}

void CanBridge::send_can_frames(){
  while(rclcpp::ok()){
      {
          // this->sendState();
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

void CanBridge::handle_can_frame(struct can_frame frame){
  // Handle the received CAN frame
  RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id);
  // Add your handling logic here
  switch(frame.can_id){
    // case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:{
    //   autonomous_temporary_acu_ms_t acu_ms_msg;
    //   autonomous_temporary_acu_ms_unpack(&acu_ms_msg, frame.data, frame.can_dlc);
    //   autonomous_temporary_jetson_ms_t jetson_ms_msg;
    //   jetson_ms_msg.mission_select = acu_ms_msg.mission_select;
    //   struct can_frame jetson_ms_frame;
    //   this->send_can_frame(jetson_ms_frame);
    //   int pack_len = autonomous_temporary_jetson_ms_pack(
    //       jetson_ms_frame.data,
    //       &jetson_ms_msg,
    //       sizeof(jetson_ms_frame.data));
    //   if (pack_len < 0) {
    //       RCLCPP_ERROR(this->get_logger(), "Failed to pack jetson_ms message: %d", pack_len);
    //       break;
    //   }
    //   jetson_ms_frame.can_id  = AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID;
    //   jetson_ms_frame.can_dlc = static_cast<uint8_t>(pack_len);
    //   this->send_can_frame(jetson_ms_frame);

    //   /* EXAMPLE USAGE OF THE ROS MESSAGES GENERATED FROM THE CAN DBC
    //   lart_msgs::msg::AcuMs acu_ms_ros_msg;
    //   acu_ms_ros_msg.header.stamp = this->now();
    //   acu_ms_ros_msg.mission_select = acu_ms_msg.mission_select;
    //   */

    //   break;
    // }
    // case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:{
    //   autonomous_temporary_res_t res_msg;
    //   autonomous_temporary_res_unpack(&res_msg, frame.data, frame.can_dlc);
    //   lart_msgs::msg::State state_msg;
    //   state_msg.header.stamp = this->now();
    //   if (res_msg.signal == 5 || res_msg.signal == 7) {
    //     state_msg.data = lart_msgs::msg::State::DRIVING;
    //   }else if (res_msg.signal == 0){
    //     state_msg.data = lart_msgs::msg::State::EMERGENCY;
    //   }
    //   // this->state_pub->publish(state_msg);
    //   break;
    // }
    // case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:{
    //   autonomous_temporary_acu_ign_t acu_ign_msg;
    //   autonomous_temporary_acu_ign_unpack(&acu_ign_msg, frame.data, frame.can_dlc);
    //   if(acu_ign_msg.asms==1 && !this->nodes_initialized){
    //     // initialize the AS nodes
    //   }
      
    //   if(acu_ign_msg.asms == 1 && acu_ign_msg.ign == 1){
    //     //Start recording bag -> send service call to the bag recorder composable node
    //   }
    //   break;
    // }
    case AUTONOMOUS_T26_ACU_FRAME_ID:{
      autonomous_t26_acu_t acu_msg;
      autonomous_t26_acu_unpack(&acu_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Acu acu_ros_msg;
      acu_ros_msg.header.stamp = this->now();
      acu_ros_msg.assi_state = acu_msg.assi_state;
      acu_ros_msg.acu_state = acu_msg.acu_state;
      acu_ros_msg.acu_cpu_temp = acu_msg.acu_cpu_temp;
      acu_ros_msg.mission_select = acu_msg.mission_select;
      acu_ros_msg.as_state = acu_msg.as_state;
      acu_ros_msg.asms = acu_msg.asms;
      acu_ros_msg.ign = acu_msg.ign;
      acu_ros_msg.emergency_cause = acu_msg.emergency_cause;

      break;
    }
    case AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID:{
      autonomous_t26_cube_mars_feedback_t cube_mars_feedback_msg;
      autonomous_t26_cube_mars_feedback_unpack(&cube_mars_feedback_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::CubemarsFeedback cube_mars_feedback_ros_msg;
      cube_mars_feedback_ros_msg.header.stamp = this->now();
      cube_mars_feedback_ros_msg.position = cube_mars_feedback_msg.position;
      cube_mars_feedback_ros_msg.speed_rpm = cube_mars_feedback_msg.speed_rpm;
      cube_mars_feedback_ros_msg.current = cube_mars_feedback_msg.current;
      cube_mars_feedback_ros_msg.driver_temp = cube_mars_feedback_msg.driver_temp;
      cube_mars_feedback_ros_msg.error_code = cube_mars_feedback_msg.error_code;

      break;
    }
    case AUTONOMOUS_T26_AQT1_FRAME_ID:{
      autonomous_t26_aqt1_t aqt1_msg;
      autonomous_t26_aqt1_unpack(&aqt1_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt1 aqt1_ros_msg;
      aqt1_ros_msg.header.stamp = this->now();
      aqt1_ros_msg.brk_press = aqt1_msg.brk_press;
      aqt1_ros_msg.res = aqt1_msg.res;
      aqt1_ros_msg.bots = aqt1_msg.bots;

      break;
    }
    case AUTONOMOUS_T26_AQT2_FRAME_ID:{
      autonomous_t26_aqt2_t aqt2_msg;
      autonomous_t26_aqt2_unpack(&aqt2_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt2 aqt2_ros_msg;
      aqt2_ros_msg.header.stamp = this->now();
      aqt2_ros_msg.wheel_angle = aqt2_msg.wheel_angle;
      break;
    }
    case AUTONOMOUS_T26_AQT3_FRAME_ID:{
      autonomous_t26_aqt3_t aqt3_msg;
      autonomous_t26_aqt3_unpack(&aqt3_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt3 aqt3_ros_msg;
      aqt3_ros_msg.header.stamp = this->now();
      aqt3_ros_msg.wheel_angle = aqt3_msg.wheel_angle;
      break;
    }
    case AUTONOMOUS_T26_AQT4_FRAME_ID:{
      autonomous_t26_aqt4_t aqt4_msg;
      autonomous_t26_aqt4_unpack(&aqt4_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt4 aqt4_ros_msg;
      aqt4_ros_msg.header.stamp = this->now();
      aqt4_ros_msg.st_angle = aqt4_msg.st_angle;
      aqt4_ros_msg.inertia = aqt4_msg.inertia;
      aqt4_ros_msg.emer_button = aqt4_msg.emer_button;
      break;
    }
    case AUTONOMOUS_T26_AQT7_FRAME_ID:{
      autonomous_t26_aqt7_t aqt7_msg;
      autonomous_t26_aqt7_unpack(&aqt7_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt7 aqt7_ros_msg;
      aqt7_ros_msg.header.stamp = this->now();
      aqt7_ros_msg.brk_press = aqt7_msg.brk_press;
      break;
    }
    case AUTONOMOUS_T26_VCU_IGN_R2_D_FRAME_ID:{
      autonomous_t26_vcu_ign_r2_d_t vcu_ign_r2_d_msg;
      autonomous_t26_vcu_ign_r2_d_unpack(&vcu_ign_r2_d_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::VcuIgnR2d vcu_ign_r2_d_ros_msg;
      vcu_ign_r2_d_ros_msg.header.stamp = this->now();
      vcu_ign_r2_d_ros_msg.ignition_manual = vcu_ign_r2_d_msg.ignition_manual;
      vcu_ign_r2_d_ros_msg.r2d_manual = vcu_ign_r2_d_msg.r2d_manual;
      vcu_ign_r2_d_ros_msg.ignition_auto = vcu_ign_r2_d_msg.ignition_auto;
      vcu_ign_r2_d_ros_msg.r2d_auto = vcu_ign_r2_d_msg.r2d_auto;
      vcu_ign_r2_d_ros_msg.shutdown_signal = vcu_ign_r2_d_msg.shutdown_signal;
      vcu_ign_r2_d_ros_msg.vcu_state = vcu_ign_r2_d_msg.vcu_state;
      vcu_ign_r2_d_ros_msg.r2d_button_raw = vcu_ign_r2_d_msg.r2_d_button_raw;
      vcu_ign_r2_d_ros_msg.ignition_switch_raw = vcu_ign_r2_d_msg.ignition_switch_raw;
      
      break;
    }
    case AUTONOMOUS_T26_VCU_HV_FRAME_ID:{
      autonomous_t26_vcu_hv_t vcu_hv_msg;
      autonomous_t26_vcu_hv_unpack(&vcu_hv_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::VcuHv vcu_hv_ros_msg;
      vcu_hv_ros_msg.header.stamp = this->now();
      vcu_hv_ros_msg.hv = vcu_hv_msg.hv;
      vcu_hv_ros_msg.brake_pressure_front = vcu_hv_msg.brake_pressure_front;
      vcu_hv_ros_msg.brake_pressure_rear = vcu_hv_msg.brake_pressure_rear;

      break;
    }
    case AUTONOMOUS_T26_VCU_RPM_FRAME_ID:{
      autonomous_t26_vcu_rpm_t vcu_rpm_msg;
      autonomous_t26_vcu_rpm_unpack(&vcu_rpm_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::VcuRpm vcu_rpm_ros_msg;
      vcu_rpm_ros_msg.header.stamp = this->now();
      vcu_rpm_ros_msg.rpm_actual = vcu_rpm_msg.rpm_actual;
      break;
    }
    case AUTONOMOUS_T26_RES_FRAME_ID:{
      autonomous_t26_res_t res_msg;
      autonomous_t26_res_unpack(&res_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Res res_ros_msg;
      res_ros_msg.header.stamp = this->now();
      res_ros_msg.signal = res_msg.signal;
      break;
    }
  }
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}

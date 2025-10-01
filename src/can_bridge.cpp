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
    case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:{
      autonomous_temporary_acu_ms_t acu_ms_msg;
      autonomous_temporary_acu_ms_unpack(&acu_ms_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID:{
      autonomous_temporary_vcu_rpm_t vcu_rpm_msg;
      autonomous_temporary_vcu_rpm_unpack(&vcu_rpm_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:{
      autonomous_temporary_acu_ign_t acu_ign_msg;
      autonomous_temporary_acu_ign_unpack(&acu_ign_msg, frame.data, frame.can_dlc);
      if(acu_ign_msg.asms==1 && !this->nodes_initialized){
        // initialize the AS nodes
      }
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:{
      autonomous_temporary_vcu_hv_t vcu_hv_msg;
      autonomous_temporary_vcu_hv_unpack(&vcu_hv_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:{
      autonomous_temporary_res_t res_msg;
      autonomous_temporary_res_unpack(&res_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG1_FRAME_ID:{
      autonomous_temporary_dyn_front_sig1_t dyn_front_sig1_msg;
      autonomous_temporary_dyn_front_sig1_unpack(&dyn_front_sig1_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG2_FRAME_ID:{
      autonomous_temporary_dyn_front_sig2_t dyn_front_sig2_msg;
      autonomous_temporary_dyn_front_sig2_unpack(&dyn_front_sig2_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG1_FRAME_ID:{
      autonomous_temporary_dyn_front_sig1_t dyn_rear_sig1_msg;
      autonomous_temporary_dyn_front_sig1_unpack(&dyn_rear_sig1_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG2_FRAME_ID:{
      autonomous_temporary_dyn_rear_sig2_t dyn_rear_sig2_msg;
      autonomous_temporary_dyn_rear_sig2_unpack(&dyn_rear_sig2_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ASF_SIGNALS_FRAME_ID:{
      autonomous_temporary_asf_signals_t asf_signals_msg;
      autonomous_temporary_asf_signals_unpack(&asf_signals_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_IGN_R2_D_FRAME_ID:{
      autonomous_temporary_vcu_ign_r2_d_t vcu_ign_r2_d_msg;
      autonomous_temporary_vcu_ign_r2_d_unpack(&vcu_ign_r2_d_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ACU_STATUS_FRAME_ID:{
      autonomous_temporary_acu_status_t acu_status_msg;
      autonomous_temporary_acu_status_unpack(&acu_status_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_APPS_RAW_FRAME_ID:{
      autonomous_temporary_vcu_apps_raw_t vcu_apps_raw_msg;
      autonomous_temporary_vcu_apps_raw_unpack(&vcu_apps_raw_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_STATUS_TX_FRAME_ID:{
      autonomous_temporary_maxon_status_tx_t maxon_status_tx_msg;
      autonomous_temporary_maxon_status_tx_unpack(&maxon_status_tx_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_STATUS2_TX_FRAME_ID:{
      autonomous_temporary_maxon_status2_tx_t maxon_status2_tx_msg;
      autonomous_temporary_maxon_status2_tx_unpack(&maxon_status2_tx_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_POSITION_TX_FRAME_ID:{
      autonomous_temporary_maxon_position_tx_t maxon_position_tx_msg;
      autonomous_temporary_maxon_position_tx_unpack(&maxon_position_tx_msg, frame.data, frame.can_dlc);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_VELOCITY_TX_FRAME_ID:{
      autonomous_temporary_maxon_velocity_tx_t maxon_velocity_tx_msg;
      autonomous_temporary_maxon_velocity_tx_unpack(&maxon_velocity_tx_msg, frame.data, frame.can_dlc);
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

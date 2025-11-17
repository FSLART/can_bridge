#include "can_bridge/can_bridge.hpp"
using std::placeholders::_1;

bool maxon_offset_defined = false;
long maxon_offset;

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

  //Initiate publishers
  this->state_pub = this->create_publisher<lart_msgs::msg::State>("/state/acu", 10);
  this->mission_pub = this->create_publisher<lart_msgs::msg::Mission>("/mission/acu", 10);
  this->dynamics_pub = this->create_publisher<lart_msgs::msg::Dynamics>("/dynamics", 10);
  this->vcu_rpm_pub = this->create_publisher<lart_msgs::msg::VcuRpm>("/vcu_rpm",10);
  this->vcu_hv_pub = this->create_publisher<lart_msgs::msg::VcuHv>("/vcuHv",10);
  this->dyn_front_sig1_pub = this->create_publisher<lart_msgs::msg::DynFrontSig1>("/dynfrontsig1",10);
  this->dyn_front_sig2_pub = this->create_publisher<lart_msgs::msg::DynFrontSig2>("/dynfrontsig2",10);
  this->dyn_rear_sig1_pub = this->create_publisher<lart_msgs::msg::DynRearSig1>("/dynrearsig1",10);
  this->dyn_rear_sig2_pub = this->create_publisher<lart_msgs::msg::DynRearSig2>("/dynrearsig2",10);
  this->asf_signals_pub = this->create_publisher<lart_msgs::msg::AsfSignals>("/asfsignals",10);
  this->vcu_ign_r2_d_pub = this->create_publisher<lart_msgs::msg::VcuIgnR2d>("/vcuignr2d",10);
  this->acu_status_pub = this->create_publisher<lart_msgs::msg::AcuStatus>("/acustatus",10);
  this->maxon_status_tx_pub = this->create_publisher<lart_msgs::msg::MaxonStatusTx>("/maxonstatustx",10);
  this->maxon_status2_tx_pub = this->create_publisher<lart_msgs::msg::MaxonStatus2Tx>("/maxonstatus2tx",10);
  this->maxon_position_tx_pub = this->create_publisher<lart_msgs::msg::MaxonPositionTx>("/maxonpositiontx",10);
  this->maxon_velocity_tx_pub = this->create_publisher<lart_msgs::msg::MaxonVelocityTx>("/maxonvelocitytx",10);

  //Initiate Subscribers
  //verificar
  this->state_sub = this->create_subscription<lart_msgs::msg::State>("/state",10,std::bind(&CanBridge::StateCallBack,this,_1)); 
  this->ekf_state_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/PoseStamped",10,std::bind(&CanBridge::ekfStateCallback,this,_1));
  this->ekf_stats_sub = this->create_subscription<lart_msgs::msg::SlamStats>("/SlamStats",10,std::bind(&CanBridge::ekfStatsCallback,this,_1));
  this->control_sub = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/DynamicsCMD",10,std::bind(&CanBridge::ControlCallback,this,_1));
  this->accelerations_sub = this->create_subscription<lart_msgs::msg::vector3Stamped>("/vector3Stamped",10,std::bind(&CanBridge::accelerationsCallback,this,_1));

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

//verificar
void CanBridge::StateCallBack(const lart_msgs::msg::State::SharedPtr msg){
  autonomous_temporary_as_state_t as_state_msg;
  as_state_msg.state = msg->data;
  struct can_frame as_state_frame; //duvida isto existe ou posso dar um nome qualquer?
  int pack_len = autonomous_temporary_as_state_pack(as_state_frame.data,&as_state_msg,sizeof(as_state_msg));
  if(pack_len < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to pack as_state message: %d", pack_len);
  }
  as_state_frame.can_id = AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID;
  as_state_frame.can_dlc = AUTONOMOUS_TEMPORARY_AS_STATE_LENGTH;
  this->send_can_frame(as_state_frame);
}

void CanBridge::ekfStateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg){
  autonomous_temporary_jetson_debug_t jetson_debug_msg;
  jetson_debug_msg.pos_x = msg->pose.position.x;
  jetson_debug_msg.pos_y = msg->pose.position.y;
  struct can_frame ekf_state_frame;
  std::ifstream file("/sys/devices/virtual/thermal/thermal_zone1/temp");
  int temp;
  file >> temp;
  float final_temp = temp / 1000.0 ;
  jetson_debug_msg.temperature = final_temp;
  int pack_len = autonomous_temporary_jetson_debug_pack(ekf_state_frame.data,&jetson_debug_msg,sizeof(jetson_debug_msg));
  if(pack_len < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to pack EkfState message: %d", pack_len);
    //duvida aqui n devia sair?
  }
  ekf_state_frame.can_id = AUTONOMOUS_TEMPORARY_JETSON_DEBUG_FRAME_ID;
  ekf_state_frame.can_dlc = AUTONOMOUS_TEMPORARY_JETSON_DEBUG_LENGTH;
  this->send_can_frame(ekf_state_frame); 
}

void CanBridge::ekfStatsCallback(const lart_msgs::msg::SlamStats::SharedPtr msg){
  (void)msg;
  autonomous_temporary_jetson_data_1_t  jetson_data_1_msg;
  struct can_frame ekf_stats_frame;
  //not filled
  jetson_data_1_msg.actual_angle = 0;
  jetson_data_1_msg.actual_speed = 0;
  jetson_data_1_msg.current_cone_count = 0;
  jetson_data_1_msg.lap_count = 0;
  jetson_data_1_msg.target_angle = 0;
  jetson_data_1_msg.target_speed = 0;
  jetson_data_1_msg.total_cone_count = 0;
  
  int pack_len = autonomous_temporary_jetson_data_1_pack(ekf_stats_frame.data,&jetson_data_1_msg,sizeof(jetson_data_1_msg));
  if(pack_len < 0){
    RCLCPP_ERROR(this->get_logger(), "Failed to pack EkfStats message: %d", pack_len);
  }
  ekf_stats_frame.can_id AUTONOMOUS_TEMPORARY_JETSON_DATA_1_FRAME_ID;
  ekf_stats_frame.can_dlc = AUTONOMOUS_TEMPORARY_JETSON_DATA_1_LENGTH;
  
  this->send_can_frame(ekf_stats_frame);
}

//Verificar e acabar!!!!
void CanBridge::ControlCallback(conts lart_msgs::msg::DynamicsCMD::SharedPtr msg){
  autonomous_temporary_rpm_target_t control_msg;
  struct can_frame control_frame;
  control_frame.can_id = AUTONOMOUS_TEMPORARY_RPM_TARGET_FRAME_ID; 
  control_frame.can_dlc = 2;
  autonomous_temporary_rpm_target_pack(control_frame.data,&control_msg, sizeof(control_msg));
  this->send_can_frame(control_frame);  

  //Target Position Frame
  struct can_frame target_position_frame;
  target_position_frame = positionToMaxonCmd(maxon_offset,msg.steering_angle);
  this->send_can_frame(target_position_frame);
}

void CanBridge::accelerationsCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg){
  //to do

}

void CanBridge::handle_can_frame(struct can_frame frame){
  // Handle the received CAN frame
  RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id);
  // Add your handling logic here
  switch(frame.can_id){
    case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:{
      autonomous_temporary_acu_ms_t acu_ms_msg;
      autonomous_temporary_acu_ms_unpack(&acu_ms_msg, frame.data, frame.can_dlc);
      autonomous_temporary_jetson_ms_t jetson_ms_msg;
      jetson_ms_msg.mission_select = acu_ms_msg.mission_select;
      struct can_frame jetson_ms_frame;
      int pack_len = autonomous_temporary_jetson_ms_pack(
          jetson_ms_frame.data,
          &jetson_ms_msg,
          sizeof(jetson_ms_frame.data));
      if (pack_len < 0) {
          RCLCPP_ERROR(this->get_logger(), "Failed to pack jetson_ms message: %d", pack_len);
          break;
      }
      jetson_ms_frame.can_id  = AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID;
      jetson_ms_frame.can_dlc = static_cast<uint8_t>(pack_len);
      this->send_can_frame(jetson_ms_frame);

      /* EXAMPLE USAGE OF THE ROS MESSAGES GENERATED FROM THE CAN DBC
      lart_msgs::msg::AcuMs acu_ms_ros_msg;
      acu_ms_ros_msg.header.stamp = this->now();
      acu_ms_ros_msg.mission_select = acu_ms_msg.mission_select;
      */

      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID:{
      autonomous_temporary_vcu_rpm_t vcu_rpm_msg;
      autonomous_temporary_vcu_rpm_unpack(&vcu_rpm_msg, frame.data, frame.can_dlc);
      this->vcu_rpm_pub->publish(vcu_rpm_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:{
      autonomous_temporary_acu_ign_t acu_ign_msg;
      autonomous_temporary_acu_ign_unpack(&acu_ign_msg, frame.data, frame.can_dlc);
      if(acu_ign_msg.asms==1 && !this->nodes_initialized){
        // initialize the AS nodes
      }
      
      if(acu_ign_msg.asms == 1 && acu_ign_msg.ign == 1){
        //Start recording bag -> send service call to the bag recorder composable node
      }
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:{
      autonomous_temporary_vcu_hv_t vcu_hv_msg;
      autonomous_temporary_vcu_hv_unpack(&vcu_hv_msg, frame.data, frame.can_dlc);
      this->vcu_hv_pub->publish(vcu_hv_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:{
      autonomous_temporary_res_t res_msg;
      autonomous_temporary_res_unpack(&res_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::State state_msg;
      state_msg.header.stamp = this->now();
      if (res_msg.signal == 5 || res_msg.signal == 7) {
        state_msg.data = lart_msgs::msg::State::DRIVING;
      }else if (res_msg.signal == 0){
        state_msg.data = lart_msgs::msg::State::EMERGENCY;
      }
      this->state_pub->publish(state_msg);
      break;
    }
    //REVER!!!!!!
    case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG1_FRAME_ID:{
      autonomous_temporary_dyn_front_sig1_t dyn_front_sig1_msg;
      autonomous_temporary_dyn_front_sig1_unpack(&dyn_front_sig1_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Dynamics dynamics_msg;
      dynamics_msg.steering_angle = dyn_front_sig1_msg.st_angle;
            
      this->dyn_front_sig1_pub->publish(dyn_front_sig1_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG2_FRAME_ID:{
      autonomous_temporary_dyn_front_sig2_t dyn_front_sig2_msg;
      autonomous_temporary_dyn_front_sig2_unpack(&dyn_front_sig2_msg, frame.data, frame.can_dlc);
      this->dyn_front_sig2_pub->publish(dyn_front_sig2_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG1_FRAME_ID:{
      autonomous_temporary_dyn_front_sig1_t dyn_rear_sig1_msg;
      autonomous_temporary_dyn_front_sig1_unpack(&dyn_rear_sig1_msg, frame.data, frame.can_dlc);
      this->dyn_rear_sig1_pub->publish(dyn_rear_sig1_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG2_FRAME_ID:{
      autonomous_temporary_dyn_rear_sig2_t dyn_rear_sig2_msg;
      autonomous_temporary_dyn_rear_sig2_unpack(&dyn_rear_sig2_msg, frame.data, frame.can_dlc);
      this->dyn_rear_sig2_pub->publish(dyn_rear_sig2_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ASF_SIGNALS_FRAME_ID:{
      autonomous_temporary_asf_signals_t asf_signals_msg;
      autonomous_temporary_asf_signals_unpack(&asf_signals_msg, frame.data, frame.can_dlc);
      this->asf_signals_pub->publish(asf_signals_pub);
      break;
    }
    case AUTONOMOUS_TEMPORARY_VCU_IGN_R2_D_FRAME_ID:{
      autonomous_temporary_vcu_ign_r2_d_t vcu_ign_r2_d_msg;
      autonomous_temporary_vcu_ign_r2_d_unpack(&vcu_ign_r2_d_msg, frame.data, frame.can_dlc);
      this->vcu_ign_r2_d_pub->publish(vcu_ign_r2_d_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_ACU_STATUS_FRAME_ID:{
      autonomous_temporary_acu_status_t acu_status_msg;
      autonomous_temporary_acu_status_unpack(&acu_status_msg, frame.data, frame.can_dlc);
      this->acu_status_pub->publish(acu_status_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_STATUS_TX_FRAME_ID:{
      autonomous_temporary_maxon_status_tx_t maxon_status_tx_msg;
      autonomous_temporary_maxon_status_tx_unpack(&maxon_status_tx_msg, frame.data, frame.can_dlc);
      this->maxon_status_tx_pub->publish(maxon_status_tx_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_STATUS2_TX_FRAME_ID:{
      autonomous_temporary_maxon_status2_tx_t maxon_status2_tx_msg;
      autonomous_temporary_maxon_status2_tx_unpack(&maxon_status2_tx_msg, frame.data, frame.can_dlc);
      this->maxon_status2_tx_pub->publish(maxon_status2_tx_msg);
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_POSITION_TX_FRAME_ID:{
      autonomous_temporary_maxon_position_tx_t maxon_position_tx_msg;
      autonomous_temporary_maxon_position_tx_unpack(&maxon_position_tx_msg, frame.data, frame.can_dlc);
      this->maxon_position_tx_pub->publish(maxon_position_tx_msg);
      if(!maxon_offset_defined){
        maxon_offset = maxon_position_tx_msg.actual_position;
        maxon_offset_defined = true;
      }
      break;
    }
    case AUTONOMOUS_TEMPORARY_MAXON_VELOCITY_TX_FRAME_ID:{
      autonomous_temporary_maxon_velocity_tx_t maxon_velocity_tx_msg;
      autonomous_temporary_maxon_velocity_tx_unpack(&maxon_velocity_tx_msg, frame.data, frame.can_dlc);
      this->maxon_velocity_tx_pub->publish(maxon_velocity_tx_msg);
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

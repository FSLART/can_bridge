#include "can_bridge/can_bridge.hpp"
using std::placeholders::_1;

using namespace std::placeholders;

CanBridge::CanBridge() : Node("can_bridge"){
  
  // create a socket
  if ((this->s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to create socket: %s", strerror(errno));
    return;
  }

  // define can interface
  struct ifreq ifr;
  strcpy(ifr.ifr_name, CAN_INTERFACE);
  ioctl(this->s, SIOCGIFINDEX, &ifr);

  // bind the socket to the CAN interface
  struct sockaddr_can addr;
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (bind(this->s, (struct sockaddr *)&addr, sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to bind socket: %s", strerror(errno));
    exit(1);
  }

  //Initiate publishers
  this->acu_pub_ = this->create_publisher<lart_msgs::msg::Acu>(TOPIC_CAN_ACU, 10);
  // TO REMOVE
  this->acu_mission_pub_ = this->create_publisher<std_msgs::msg::Float32>("/can/dbc/acu/mission_select", 10);

  this->cube_mars_feedback_pub_ = this->create_publisher<lart_msgs::msg::CubemarsFeedback>(TOPIC_CAN_CUBEMARS_FEEDBACK, 10);
  this->res_pub_ = this->create_publisher<lart_msgs::msg::Res>(TOPIC_CAN_RES, 10);

  this->vcu_hv_pub_ = this->create_publisher<lart_msgs::msg::VcuHv>(TOPIC_CAN_VCU_HV, 10);
  this->vcu_ign_r2d_pub_ = this->create_publisher<lart_msgs::msg::VcuIgnR2d>(TOPIC_CAN_VCU_IGN_R2D, 10);
  this->vcu_rpm_pub_ = this->create_publisher<lart_msgs::msg::VcuRpm>(TOPIC_CAN_VCU_RPM, 10);

  this->aquisition1_pub_ = this->create_publisher<lart_msgs::msg::Aqt1>(TOPIC_CAN_AQUTION_AQT1, 10);
  this->aquisition2_pub_ = this->create_publisher<lart_msgs::msg::Aqt2>(TOPIC_CAN_AQUTION_AQT2, 10);
  this->aquisition3_pub_ = this->create_publisher<lart_msgs::msg::Aqt3>(TOPIC_CAN_AQUTION_AQT3, 10);
  this->aquisition4_pub_ = this->create_publisher<lart_msgs::msg::Aqt4>(TOPIC_CAN_AQUTION_AQT4, 10);
  this->aquisition7_pub_ = this->create_publisher<lart_msgs::msg::Aqt7>(TOPIC_CAN_AQUTION_AQT7, 10);
  this->asf_signals_pub_ = this->create_publisher<lart_msgs::msg::AsfSignals>("/asf", 10);

  // Initiate subscribers
  this->dv_dynamics1_sub_ = this->create_subscription<lart_msgs::msg::DvDynamics1>(TOPIC_DV_DYNAMICS1, 10, std::bind(&CanBridge::handle_dv_dynamics1_message, this, _1));
  this->dv_dynamics2_sub_ = this->create_subscription<lart_msgs::msg::DvDynamics2>(TOPIC_DV_DYNAMICS2, 10, std::bind(&CanBridge::handle_dv_dynamics2_message, this, _1));
  this->dv_status_sub_ = this->create_subscription<lart_msgs::msg::DvStatus>("/dv/status", 10, std::bind(&CanBridge::handle_dv_status_message, this, _1));

  this->jetson_sub_ = this->create_subscription<lart_msgs::msg::Jetson>(TOPIC_JETSON, 10, std::bind(&CanBridge::handle_jetson_message, this, _1));
  
  this->cubemars_position_loop_sub_ = this->create_subscription<lart_msgs::msg::CubemarsPositionLoop>(TOPIC_CUBEMARS_POSITION_LOOP, 10, std::bind(&CanBridge::handle_cubemars_position_loop_message, this, _1));

  this->vcu_torque_target_sub_ = this->create_subscription<lart_msgs::msg::VcuTorqueTarget>(TOPIC_VCU_TORQUE_TARGET, 10, std::bind(&CanBridge::handle_vcu_torque_target_message, this, _1));
  this->vcu_rpm_target_sub_ = this->create_subscription<lart_msgs::msg::VcuRpmTarget>(TOPIC_VCU_RPM_TARGET, 10, std::bind(&CanBridge::handle_vcu_rpm_target_message, this, _1));
  
  
  // create a thread to read CAN frames
  std::thread read_can_thread(&CanBridge::read_can_frame, this);
  read_can_thread.detach();
  RCLCPP_INFO(this->get_logger(), "Can Bridge Node has been started");
}

void CanBridge::read_can_frame()
{
  while (rclcpp::ok())
  {
    struct can_frame frame;
    int nbytes = read(this->s, &frame, sizeof(frame));
    if (nbytes < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to read CAN frame: %s", strerror(errno));
      return;
    }
    handle_can_frame(frame); // Send the received can frame to a function that handles it
                             // RCLCPP_INFO(this->get_logger(), "RECEIVED A CAN FRAME");
  }
}

void CanBridge::send_can_frame(struct can_frame frame)
{
  std::lock_guard<std::mutex> guard(this->socket_mutex);
  if(!this->asms) return;
  if (write(this->s, &frame, sizeof(frame)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
  }
}

void CanBridge::handle_can_frame(struct can_frame frame){
  // Handle the received CAN frame
  // RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id);
  switch(frame.can_id & CAN_EFF_MASK){
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
      acu_ros_msg.emergency = acu_msg.emergency;
      acu_ros_msg.asms = acu_msg.asms;
      acu_ros_msg.ign = acu_msg.ign;
      acu_ros_msg.emergency_cause = acu_msg.emergency_cause;
      this->acu_pub_->publish(acu_ros_msg);


      //TO REMOVE -> MESSAGE TO DASHBOARD FOR VSV
      std_msgs::msg::Float32 mission_msg;
      mission_msg.data = acu_msg.mission_select;
      this->acu_mission_pub_->publish(mission_msg);

      this->asms = acu_msg.asms; 

      break;
    }
    case AUTONOMOUS_T26_CUBE_MARS_FEEDBACK_FRAME_ID:{
      autonomous_t26_cube_mars_feedback_t cube_mars_feedback_msg;
      autonomous_t26_cube_mars_feedback_unpack(&cube_mars_feedback_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::CubemarsFeedback cube_mars_feedback_ros_msg;
      cube_mars_feedback_ros_msg.header.stamp = this->now();
      cube_mars_feedback_ros_msg.position = cube_mars_feedback_msg.position*0.1;
      cube_mars_feedback_ros_msg.speed_rpm = cube_mars_feedback_msg.speed_rpm*0.0529100529;
      cube_mars_feedback_ros_msg.current = cube_mars_feedback_msg.current*0.01;
      cube_mars_feedback_ros_msg.driver_temp = cube_mars_feedback_msg.driver_temp;
      cube_mars_feedback_ros_msg.error_code = cube_mars_feedback_msg.error_code;
      this->cube_mars_feedback_pub_->publish(cube_mars_feedback_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_AQT1_FRAME_ID:{
      autonomous_t26_aqt1_t aqt1_msg;
      autonomous_t26_aqt1_unpack(&aqt1_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt1 aqt1_ros_msg;
      aqt1_ros_msg.header.stamp = this->now();
      aqt1_ros_msg.frt_brk_press = aqt1_msg.frt_brk_press*0.1;
      aqt1_ros_msg.res = aqt1_msg.res;
      aqt1_ros_msg.bots = aqt1_msg.bots;
      this->aquisition1_pub_->publish(aqt1_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_AQT2_FRAME_ID:{
      autonomous_t26_aqt2_t aqt2_msg;
      autonomous_t26_aqt2_unpack(&aqt2_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt2 aqt2_ros_msg;
      aqt2_ros_msg.header.stamp = this->now();
      aqt2_ros_msg.wheel_spd = aqt2_msg.wheel_spd*0.1;
      this->aquisition2_pub_->publish(aqt2_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_AQT3_FRAME_ID:{
      autonomous_t26_aqt3_t aqt3_msg;
      autonomous_t26_aqt3_unpack(&aqt3_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt3 aqt3_ros_msg;
      aqt3_ros_msg.header.stamp = this->now();
      aqt3_ros_msg.wheel_spd = aqt3_msg.wheel_spd*0.1;
      this->aquisition3_pub_->publish(aqt3_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_AQT4_FRAME_ID:{
      autonomous_t26_aqt4_t aqt4_msg;
      autonomous_t26_aqt4_unpack(&aqt4_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt4 aqt4_ros_msg;
      aqt4_ros_msg.header.stamp = this->now();
      aqt4_ros_msg.st_angle = aqt4_msg.st_angle*0.1;
      aqt4_ros_msg.susp_l = aqt4_msg.susp_l*0.1;
      aqt4_ros_msg.susp_r = aqt4_msg.susp_r*0.1;
      aqt4_ros_msg.inertia = aqt4_msg.inertia;
      aqt4_ros_msg.emergency = aqt4_msg.emergency;
      this->aquisition4_pub_->publish(aqt4_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_AQT7_FRAME_ID:{
      autonomous_t26_aqt7_t aqt7_msg;
      autonomous_t26_aqt7_unpack(&aqt7_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Aqt7 aqt7_ros_msg;
      aqt7_ros_msg.header.stamp = this->now();
      aqt7_ros_msg.rear_brk_press = aqt7_msg.rear_brk_press*0.1;
      this->aquisition7_pub_->publish(aqt7_ros_msg);
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
      this->vcu_ign_r2d_pub_->publish(vcu_ign_r2_d_ros_msg);
      
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
      this->vcu_hv_pub_->publish(vcu_hv_ros_msg);
      break;
    }
    case AUTONOMOUS_T26_VCU_RPM_FRAME_ID:{
      autonomous_t26_vcu_rpm_t vcu_rpm_msg;
      autonomous_t26_vcu_rpm_unpack(&vcu_rpm_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::VcuRpm vcu_rpm_ros_msg;
      vcu_rpm_ros_msg.header.stamp = this->now();
      vcu_rpm_ros_msg.motor_rpm_left = vcu_rpm_msg.motor_rpm_left;
      vcu_rpm_ros_msg.motor_rpm_right = vcu_rpm_msg.motor_rpm_right;
      vcu_rpm_ros_msg.motor_current_left = vcu_rpm_msg.motor_current_left;
      vcu_rpm_ros_msg.motor_current_right = vcu_rpm_msg.motor_current_right;
      this->vcu_rpm_pub_->publish(vcu_rpm_ros_msg);
      break;
    }
    case 0x181:{
      autonomous_t26_res_t res_msg;
      autonomous_t26_res_unpack(&res_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::Res res_ros_msg;
      res_ros_msg.header.stamp = this->now();
      res_ros_msg.signal = res_msg.signal;
      this->res_pub_->publish(res_ros_msg);
      break;
    }

    case AUTONOMOUS_T26_ASF_SIGNALS_FRAME_ID:{
      autonomous_t26_asf_signals_t asf_signals_msg;
      autonomous_t26_asf_signals_unpack(&asf_signals_msg, frame.data, frame.can_dlc);
      lart_msgs::msg::AsfSignals asf_signals_ros_msg;
      asf_signals_ros_msg.header.stamp = this->now();
      asf_signals_ros_msg.ebs_pressure_tank_front = asf_signals_msg.ebs_pressure_tank_front*0.1;
      asf_signals_ros_msg.ebs_pressure_tank_rear = asf_signals_msg.ebs_pressure_tank_rear*0.1;
      asf_signals_ros_msg.brake_pressure_front = asf_signals_msg.brake_pressure_front*0.1;
      asf_signals_ros_msg.brake_pressure_rear = asf_signals_msg.brake_pressure_rear*0.1;
      this->asf_signals_pub_->publish(asf_signals_ros_msg);
      break;
    }
  }
}

void CanBridge::handle_vcu_torque_target_message(const lart_msgs::msg::VcuTorqueTarget::SharedPtr msg){
  autonomous_t26_vcu_torque_target_t vcu_torque_target_msg;
  vcu_torque_target_msg.torque_target = msg->torque_target;

  struct can_frame vcu_torque_target_frame;
  int pack_len = autonomous_t26_vcu_torque_target_pack(
      vcu_torque_target_frame.data,
      &vcu_torque_target_msg,
      sizeof(vcu_torque_target_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack vcu_torque_target message: %d", pack_len);
      return;
  }
  vcu_torque_target_frame.can_id  = AUTONOMOUS_T26_VCU_TORQUE_TARGET_FRAME_ID;
  vcu_torque_target_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(vcu_torque_target_frame);
}

void CanBridge::handle_vcu_rpm_target_message(const lart_msgs::msg::VcuRpmTarget::SharedPtr msg){
  autonomous_t26_vcu_rpm_target_t vcu_rpm_target_msg;
  vcu_rpm_target_msg.rpm_target = msg->rpm_target;

  struct can_frame vcu_rpm_target_frame;
  int pack_len = autonomous_t26_vcu_rpm_target_pack(
      vcu_rpm_target_frame.data,
      &vcu_rpm_target_msg,
      sizeof(vcu_rpm_target_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack vcu_rpm_target message: %d", pack_len);
      return;
  }
  vcu_rpm_target_frame.can_id  = AUTONOMOUS_T26_VCU_RPM_TARGET_FRAME_ID;
  vcu_rpm_target_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(vcu_rpm_target_frame);
}

void CanBridge::handle_cubemars_position_loop_message(const lart_msgs::msg::CubemarsPositionLoop::SharedPtr msg){
  autonomous_t26_cube_mars_position_loop_t cubemars_position_loop_msg;
  cubemars_position_loop_msg.position = msg->position/0.0001;

  struct can_frame cubemars_position_loop_frame;
  int pack_len = autonomous_t26_cube_mars_position_loop_pack(
      cubemars_position_loop_frame.data,
      &cubemars_position_loop_msg,
      sizeof(cubemars_position_loop_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack cubemars_position_loop message: %d", pack_len);
      return;
  }
  cubemars_position_loop_frame.can_id  = AUTONOMOUS_T26_CUBE_MARS_POSITION_LOOP_FRAME_ID | CAN_EFF_FLAG;
  cubemars_position_loop_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(cubemars_position_loop_frame);
}

void CanBridge::handle_dv_dynamics1_message(const lart_msgs::msg::DvDynamics1::SharedPtr msg){
  autonomous_t26_dv_dynamics_1_t dv_dynamics1_msg;
  dv_dynamics1_msg.speed_actual = msg->speed_actual;
  dv_dynamics1_msg.speed_target = msg->speed_target;
  dv_dynamics1_msg.steering_angle_actual = msg->steering_angle_actual/0.5;
  dv_dynamics1_msg.steering_angle_target = msg->steering_angle_target/0.5;
  dv_dynamics1_msg.brake_hydr_actual = msg->brake_hydr_actual;
  dv_dynamics1_msg.brake_hydr_target = msg->brake_hydr_target;
  dv_dynamics1_msg.motor_moment_actual = msg->motor_moment_actual;
  dv_dynamics1_msg.motor_moment_target = msg->motor_moment_target;

  struct can_frame dv_dynamics1_frame;
  int pack_len = autonomous_t26_dv_dynamics_1_pack(
      dv_dynamics1_frame.data,
      &dv_dynamics1_msg,
      sizeof(dv_dynamics1_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack dv_dynamics1 message: %d", pack_len);
      return;
  }
  dv_dynamics1_frame.can_id  = AUTONOMOUS_T26_DV_DYNAMICS_1_FRAME_ID;
  dv_dynamics1_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(dv_dynamics1_frame);
}

void CanBridge::handle_dv_dynamics2_message(const lart_msgs::msg::DvDynamics2::SharedPtr msg){
  autonomous_t26_dv_dynamics_2_t dv_dynamics2_msg;
  dv_dynamics2_msg.acceleration_longitudinal = msg->acceleration_longitudinal/0.001953125;
  dv_dynamics2_msg.acceleration_lateral = msg->acceleration_lateral/0.001953125;
  dv_dynamics2_msg.yaw_rate = msg->yaw_rate/0.0078125;

  struct can_frame dv_dynamics2_frame;
  int pack_len = autonomous_t26_dv_dynamics_2_pack(
      dv_dynamics2_frame.data,
      &dv_dynamics2_msg,
      sizeof(dv_dynamics2_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack dv_dynamics2 message: %d", pack_len);
      return;
  }
  dv_dynamics2_frame.can_id  = AUTONOMOUS_T26_DV_DYNAMICS_2_FRAME_ID;
  dv_dynamics2_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(dv_dynamics2_frame);
}

void CanBridge::handle_dv_status_message(const lart_msgs::msg::DvStatus::SharedPtr msg){
  autonomous_t26_dv_status_t dv_status_msg;
  dv_status_msg.as_status = msg->as_status;
  dv_status_msg.asb_ebs_state = msg->asb_ebs_state;
  dv_status_msg.ami_state = msg->ami_state;
  dv_status_msg.steering_state = msg->steering_state;
  dv_status_msg.asb_redundancy_state = msg->asb_redundancy_state;
  dv_status_msg.lap_counter = msg->lap_counter;
  dv_status_msg.cones_count_actual = msg->cones_count_actual;
  dv_status_msg.cones_count_all = msg->cones_count_all;

  struct can_frame dv_status_frame;
  int pack_len = autonomous_t26_dv_status_pack(
      dv_status_frame.data,
      &dv_status_msg,
      sizeof(dv_status_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack dv_status message: %d", pack_len);
      return;
  }
  dv_status_frame.can_id  = AUTONOMOUS_T26_DV_STATUS_FRAME_ID;
  dv_status_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(dv_status_frame);
}

void CanBridge::handle_jetson_message(const lart_msgs::msg::Jetson::SharedPtr msg){
  autonomous_t26_jetson_t jetson_msg;
  jetson_msg.as_state = msg->as_state;
  jetson_msg.as_mission = msg->as_mission;
  jetson_msg.temperature = msg->temperature;
  jetson_msg.cpu = msg->cpu;
  jetson_msg.gpu = msg->gpu;

  struct can_frame jetson_frame;
  int pack_len = autonomous_t26_jetson_pack(
      jetson_frame.data,
      &jetson_msg,
      sizeof(jetson_frame.data));
  if (pack_len < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack jetson message: %d", pack_len);
      return;
  }
  jetson_frame.can_id  = AUTONOMOUS_T26_JETSON_FRAME_ID;
  jetson_frame.can_dlc = static_cast<uint8_t>(pack_len);
  this->send_can_frame(jetson_frame);
}

int main(int argc, char ** argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}

#include "can_bridge/can_bridge.hpp"
using std::placeholders::_1;

CanBridge::CanBridge() : Node("can_bridge")
{
  RCLCPP_INFO(this->get_logger(), "Can Bridge Node has been started");

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

  // Initiate publishers
  this->state_pub = this->create_publisher<lart_msgs::msg::State>("/state/acu", 10);
  this->mission_pub = this->create_publisher<lart_msgs::msg::Mission>("/mission/acu", 10);
  this->dynamics_pub = this->create_publisher<lart_msgs::msg::Dynamics>("/dynamics", 10);
  this->vcu_rpm_pub = this->create_publisher<lart_msgs::msg::VcuRpm>("/vcu_rpm", 10);
  this->vcu_hv_pub = this->create_publisher<lart_msgs::msg::VcuHv>("/vcuHv", 10);
  this->dyn_front_sig1_pub = this->create_publisher<lart_msgs::msg::DynFrontSig1>("/dynfrontsig1", 10);
  this->dyn_front_sig2_pub = this->create_publisher<lart_msgs::msg::DynFrontSig2>("/dynfrontsig2", 10);
  this->dyn_rear_sig1_pub = this->create_publisher<lart_msgs::msg::DynRearSig1>("/dynrearsig1", 10);
  this->dyn_rear_sig2_pub = this->create_publisher<lart_msgs::msg::DynRearSig2>("/dynrearsig2", 10);
  this->asf_signals_pub = this->create_publisher<lart_msgs::msg::AsfSignals>("/asfsignals", 10);
  this->vcu_ign_r2_d_pub = this->create_publisher<lart_msgs::msg::VcuIgnR2d>("/vcuignr2d", 10);
  this->acu_status_pub = this->create_publisher<lart_msgs::msg::AcuStatus>("/acustatus", 10);
  this->maxon_status_tx_pub = this->create_publisher<lart_msgs::msg::MaxonStatusTx>("/maxonstatustx", 10);
  this->maxon_status2_tx_pub = this->create_publisher<lart_msgs::msg::MaxonStatus2Tx>("/maxonstatus2tx", 10);
  this->maxon_position_tx_pub = this->create_publisher<lart_msgs::msg::MaxonPositionTx>("/maxonpositiontx", 10);
  this->maxon_velocity_tx_pub = this->create_publisher<lart_msgs::msg::MaxonVelocityTx>("/maxonvelocitytx", 10);

  // Initiate Subscribers
  // verificar
  this->state_sub = this->create_subscription<lart_msgs::msg::State>("/state", 10, std::bind(&CanBridge::StateCallBack, this, _1));
  this->ekf_state_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/PoseStamped", 10, std::bind(&CanBridge::ekfStateCallback, this, _1));
  this->ekf_stats_sub = this->create_subscription<lart_msgs::msg::SlamStats>("/SlamStats", 10, std::bind(&CanBridge::ekfStatsCallback, this, _1));
  this->control_sub = this->create_subscription<lart_msgs::msg::DynamicsCMD>("/DynamicsCMD", 10, std::bind(&CanBridge::ControlCallback, this, _1));
  this->accelerations_sub = this->create_subscription<geometry_msgs::msg::Vector3Stamped>("/imu/acceleration", 10, std::bind(&CanBridge::accelerationsCallback, this, _1));

  // create a thread to read CAN frames
  std::thread read_can_thread(&CanBridge::read_can_frame, this);
  read_can_thread.detach();

  std::thread send_can_thread(&CanBridge::send_can_frames, this);
  send_can_thread.detach();
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
  if (write(this->s, &frame, sizeof(frame)) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to send CAN frame: %s", strerror(errno));
  }
}

void CanBridge::send_can_frames()
{
  while (rclcpp::ok())
  {
    {
      // this->sendState();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

// verificar
void CanBridge::StateCallBack(const lart_msgs::msg::State::SharedPtr msg)
{
  autonomous_temporary_as_state_t as_state_msg;
  as_state_msg.state = msg->data;
  struct can_frame as_state_frame; // duvida isto existe ou posso dar um nome qualquer?
  int pack_len = autonomous_temporary_as_state_pack(as_state_frame.data, &as_state_msg, sizeof(as_state_msg));
  if (pack_len < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to pack as_state message: %d", pack_len);
  }
  as_state_frame.can_id = AUTONOMOUS_TEMPORARY_AS_STATE_FRAME_ID;
  as_state_frame.can_dlc = AUTONOMOUS_TEMPORARY_AS_STATE_LENGTH;
  this->send_can_frame(as_state_frame);
}

void CanBridge::ekfStateCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  autonomous_temporary_jetson_debug_t jetson_debug_msg;
  jetson_debug_msg.pos_x = msg->pose.position.x;
  jetson_debug_msg.pos_y = msg->pose.position.y;
  struct can_frame ekf_state_frame;
  std::ifstream file("/sys/devices/virtual/thermal/thermal_zone1/temp");
  int temp;
  file >> temp;
  float final_temp = temp / 1000.0;
  jetson_debug_msg.temperature = final_temp;
  int pack_len = autonomous_temporary_jetson_debug_pack(ekf_state_frame.data, &jetson_debug_msg, sizeof(jetson_debug_msg));
  if (pack_len < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to pack EkfState message: %d", pack_len);
    // duvida aqui n devia sair?
  }
  ekf_state_frame.can_id = AUTONOMOUS_TEMPORARY_JETSON_DEBUG_FRAME_ID;
  ekf_state_frame.can_dlc = AUTONOMOUS_TEMPORARY_JETSON_DEBUG_LENGTH;
  this->send_can_frame(ekf_state_frame);
}

void CanBridge::ekfStatsCallback(const lart_msgs::msg::SlamStats::SharedPtr msg)
{
  (void)msg;
  autonomous_temporary_jetson_data_1_t jetson_data_1_msg;
  struct can_frame ekf_stats_frame;
  // not filled
  jetson_data_1_msg.actual_angle = 0;
  jetson_data_1_msg.actual_speed = 0;
  jetson_data_1_msg.current_cone_count = 0;
  jetson_data_1_msg.lap_count = 0;
  jetson_data_1_msg.target_angle = 0;
  jetson_data_1_msg.target_speed = 0;
  jetson_data_1_msg.total_cone_count = 0;

  int pack_len = autonomous_temporary_jetson_data_1_pack(ekf_stats_frame.data, &jetson_data_1_msg, sizeof(jetson_data_1_msg));
  if (pack_len < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to pack EkfStats message: %d", pack_len);
  }
  ekf_stats_frame.can_id = AUTONOMOUS_TEMPORARY_JETSON_DATA_1_FRAME_ID;
  ekf_stats_frame.can_dlc = AUTONOMOUS_TEMPORARY_JETSON_DATA_1_LENGTH;

  this->send_can_frame(ekf_stats_frame);
}

void CanBridge::ControlCallback(const lart_msgs::msg::DynamicsCMD::SharedPtr msg)
{
  autonomous_temporary_rpm_target_t control_msg;
  struct can_frame control_frame;
  control_frame.can_id = AUTONOMOUS_TEMPORARY_RPM_TARGET_FRAME_ID;
  control_frame.can_dlc = 2;
  control_msg.rpm_target = msg->rpm;

  autonomous_temporary_rpm_target_pack(control_frame.data, &control_msg, sizeof(control_msg));
  this->send_can_frame(control_frame);

  // Target Position Frame
  struct can_frame target_position_frame;
  target_position_frame = positionToMaxonCmd(maxon_offset, msg->steering_angle);
  this->send_can_frame(target_position_frame);
}

void CanBridge::accelerationsCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
{
  (void)msg;
}

void CanBridge::service_bag_start()
{
  if (!recordBag_service_activated)
  {
    // mudar de nomes das variaveis e verificar os tipos
    this->start_recording_bag = this->create_client<std_srvs::srv::Trigger>("/start_recording");

    while (!this->start_recording_bag->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /start_recording service.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /start_recording service...");
    }

    this->stop_recording_bag = this->create_client<std_srvs::srv::Trigger>("/stop_recording"); // Check this message

    while (!this->stop_recording_bag->wait_for_service(std::chrono::seconds(1)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for /stop_recording.");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "Waiting for /stop_recording...");
    }
  }
}

// composable pos node
void CanBridge::start_recording_request()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = this->start_recording_bag->async_send_request(
      request,
      std::bind(&CanBridge::handle_start_recording_response, this, _1));
}

void CanBridge::handle_start_recording_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  // Perguntar se o start recording envia alguma msg ou se apenas começa a gravar
  try
  {
    auto response = future.get();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }
}

void CanBridge::stop_recording_request()
{
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto response = this->stop_recording_bag->async_send_request(
      request,
      std::bind(&CanBridge::hanlde_stop_recording_response, this, _1));
}

void CanBridge::hanlde_stop_recording_response(rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future)
{
  try
  {
    auto response = future.get();
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
  }
}

void CanBridge::handle_can_frame(struct can_frame frame)
{
  // Handle the received CAN frame
  RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id);
  // Add your handling logic here
  switch (frame.can_id)
  {
  case AUTONOMOUS_TEMPORARY_ACU_MS_FRAME_ID:
  {
    autonomous_temporary_acu_ms_t acu_ms_msg;
    autonomous_temporary_acu_ms_unpack(&acu_ms_msg, frame.data, frame.can_dlc);
    autonomous_temporary_jetson_ms_t jetson_ms_msg;
    jetson_ms_msg.mission_select = acu_ms_msg.mission_select;
    struct can_frame jetson_ms_frame;
    int pack_len = autonomous_temporary_jetson_ms_pack(
        jetson_ms_frame.data,
        &jetson_ms_msg,
        sizeof(jetson_ms_frame.data));
    if (pack_len < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to pack jetson_ms message: %d", pack_len);
      break;
    }
    jetson_ms_frame.can_id = AUTONOMOUS_TEMPORARY_JETSON_MS_FRAME_ID;
    jetson_ms_frame.can_dlc = static_cast<uint8_t>(pack_len);
    this->send_can_frame(jetson_ms_frame);

    /* EXAMPLE USAGE OF THE ROS MESSAGES GENERATED FROM THE CAN DBC
    lart_msgs::msg::AcuMs acu_ms_ros_msg;
    acu_ms_ros_msg.header.stamp = this->now();
    acu_ms_ros_msg.mission_select = acu_ms_msg.mission_select;
    */

    break;
  }
  case AUTONOMOUS_TEMPORARY_VCU_RPM_FRAME_ID:
  {
    autonomous_temporary_vcu_rpm_t vcu_rpm_msg;
    autonomous_temporary_vcu_rpm_unpack(&vcu_rpm_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::VcuRpm ros_msg;
    ros_msg.rpm_actual = vcu_rpm_msg.rpm_actual;
    this->vcu_rpm_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_ACU_IGN_FRAME_ID:
  {
    autonomous_temporary_acu_ign_t acu_ign_msg;
    autonomous_temporary_acu_ign_unpack(&acu_ign_msg, frame.data, frame.can_dlc);
    if (acu_ign_msg.asms == 1 && !this->nodes_initialized)
    {
      // initialize the AS nodes
      send_can_frame(turnOnCANOpenDevicesFrame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      send_can_frame(enterPreOpModeCmdFrame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      send_can_frame(opModeCmdFrame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      send_can_frame(setVelocityCmd());
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      send_can_frame(shutdownCmdFrame);
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      send_can_frame(newValueFrame);
    }

    if (acu_ign_msg.asms == 1 && acu_ign_msg.ign == 1)
    {
      // Start recording bag -> send service call to the bag recorder composable node
      service_bag_start();
      start_recording_request();
    }
    break;
  }
  case AUTONOMOUS_TEMPORARY_VCU_HV_FRAME_ID:
  {
    autonomous_temporary_vcu_hv_t vcu_hv_msg;
    autonomous_temporary_vcu_hv_unpack(&vcu_hv_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::VcuHv ros_msg;
    ros_msg.brake_pressure_front = vcu_hv_msg.brake_pressure_front;
    ros_msg.brake_pressure_rear = vcu_hv_msg.brake_pressure_rear;
    ros_msg.hv = vcu_hv_msg.hv;
    this->vcu_hv_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_RES_FRAME_ID:
  {
    autonomous_temporary_res_t res_msg;
    autonomous_temporary_res_unpack(&res_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::State state_msg;
    state_msg.header.stamp = this->now();
    if (res_msg.signal == 5 || res_msg.signal == 7)
    {
      state_msg.data = lart_msgs::msg::State::DRIVING;
    }
    else if (res_msg.signal == 0)
    {
      state_msg.data = lart_msgs::msg::State::EMERGENCY;
    }
    this->state_pub->publish(state_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG1_FRAME_ID:
  {
    autonomous_temporary_dyn_front_sig1_t dyn_front_sig1_msg;
    autonomous_temporary_dyn_front_sig1_unpack(&dyn_front_sig1_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::DynFrontSig1 ros_msg;
    ros_msg.st_angle = dyn_front_sig1_msg.st_angle;
    ros_msg.susp_l = dyn_front_sig1_msg.susp_l;
    ros_msg.susp_r = dyn_front_sig1_msg.susp_r;
    this->dyn_front_sig1_pub->publish(ros_msg);
    if (!maxon_offset_defined && maxon_activated && maxon_initial_position_defined)
    {
      int raw_angle_pos = RAD_SW_ANGLE_TO_ACTUATOR_POS(DEG_TO_RAD(dyn_front_sig1_msg.st_angle)); // calculate the position of the maxon in encoder ticks
      maxon_offset = maxon_initial_position + raw_angle_pos;                                     // set the relative zero to the first position of the maxon when the system is turned on
      maxon_offset_defined = true;
    }
    break;
  }
  case AUTONOMOUS_TEMPORARY_DYN_FRONT_SIG2_FRAME_ID:
  {
    autonomous_temporary_dyn_front_sig2_t dyn_front_sig2_msg;
    autonomous_temporary_dyn_front_sig2_unpack(&dyn_front_sig2_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::DynFrontSig2 ros_msg;
    ros_msg.spd_left = dyn_front_sig2_msg.spd_left;
    ros_msg.spd_right = dyn_front_sig2_msg.spd_right;
    this->dyn_front_sig2_pub->publish(ros_msg);
    break;
  }

  case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG1_FRAME_ID:
  {
    autonomous_temporary_dyn_front_sig1_t dyn_rear_sig1_msg;
    autonomous_temporary_dyn_front_sig1_unpack(&dyn_rear_sig1_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::DynRearSig1 ros_msg;
    ros_msg.brk_press = dyn_rear_sig1_msg.st_angle; // mostrar isto ao ANDRE!!
    ros_msg.susp_l = dyn_rear_sig1_msg.susp_l;
    ros_msg.susp_r = dyn_rear_sig1_msg.susp_r;
    this->dyn_rear_sig1_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_DYN_REAR_SIG2_FRAME_ID:
  {
    autonomous_temporary_dyn_rear_sig2_t dyn_rear_sig2_msg;
    autonomous_temporary_dyn_rear_sig2_unpack(&dyn_rear_sig2_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::DynRearSig2 ros_msg;
    ros_msg.spd_left = dyn_rear_sig2_msg.spd_left;
    ros_msg.spd_right = dyn_rear_sig2_msg.spd_right;
    this->dyn_rear_sig2_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_ASF_SIGNALS_FRAME_ID:
  {
    autonomous_temporary_asf_signals_t asf_signals_msg;
    autonomous_temporary_asf_signals_unpack(&asf_signals_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::AsfSignals ros_msg;
    ros_msg.brake_pressure_front = asf_signals_msg.brake_pressure_front;
    ros_msg.brake_pressure_rear = asf_signals_msg.brake_pressure_rear;
    ros_msg.ebs_pressure_tank_front = asf_signals_msg.ebs_pressure_tank_front;
    ros_msg.ebs_pressure_tank_rear = asf_signals_msg.ebs_pressure_tank_rear;
    this->asf_signals_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_VCU_IGN_R2_D_FRAME_ID:
  {
    autonomous_temporary_vcu_ign_r2_d_t vcu_ign_r2_d_msg;
    autonomous_temporary_vcu_ign_r2_d_unpack(&vcu_ign_r2_d_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::VcuIgnR2d ros_msg;
    ros_msg.ignition_auto = vcu_ign_r2_d_msg.ignition_auto;
    ros_msg.ignition_manual = vcu_ign_r2_d_msg.ignition_manual;
    ros_msg.ignition_switch_raw = vcu_ign_r2_d_msg.ignition_switch_raw;
    ros_msg.r2d_button_raw = vcu_ign_r2_d_msg.r2_d_button_raw;
    ros_msg.r2d_auto = vcu_ign_r2_d_msg.r2d_auto;
    ros_msg.r2d_manual = vcu_ign_r2_d_msg.r2d_manual;
    ros_msg.shutdown_signal = vcu_ign_r2_d_msg.shutdown_signal;
    ros_msg.vcu_state = vcu_ign_r2_d_msg.vcu_state;
    this->vcu_ign_r2_d_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_ACU_STATUS_FRAME_ID:
  {
    autonomous_temporary_acu_status_t acu_status_msg;
    autonomous_temporary_acu_status_unpack(&acu_status_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::AcuStatus ros_msg;
    ros_msg.acu_state = acu_status_msg.acu_state;
    ros_msg.assi_state = acu_status_msg.assi_state;
    ros_msg.internal_temperature = acu_status_msg.internal_temperature;
    this->acu_status_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_MAXON_STATUS_TX_FRAME_ID:
  {
    autonomous_temporary_maxon_status_tx_t maxon_status_tx_msg;
    autonomous_temporary_maxon_status_tx_unpack(&maxon_status_tx_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::MaxonStatusTx ros_msg;
    ros_msg.control_word = maxon_status_tx_msg.control_word;
    ros_msg.current_actual_value = maxon_status_tx_msg.current_actual_value;
    ros_msg.status_word = maxon_status_tx_msg.status_word;
    this->maxon_status_tx_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_MAXON_STATUS2_TX_FRAME_ID:
  {
    autonomous_temporary_maxon_status2_tx_t maxon_status2_tx_msg;
    autonomous_temporary_maxon_status2_tx_unpack(&maxon_status2_tx_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::MaxonStatus2Tx ros_msg;
    ros_msg.error_code = maxon_status2_tx_msg.error_code;
    ros_msg.status_word = maxon_status2_tx_msg.status_word;
    ros_msg.current_average_value = maxon_status2_tx_msg.current_average_value;

    this->maxon_status2_tx_pub->publish(ros_msg);
    break;
  }
  case AUTONOMOUS_TEMPORARY_MAXON_POSITION_TX_FRAME_ID:
  {
    autonomous_temporary_maxon_position_tx_t maxon_position_tx_msg;
    autonomous_temporary_maxon_position_tx_unpack(&maxon_position_tx_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::MaxonPositionTx ros_msg;
    ros_msg.status_word = maxon_position_tx_msg.status_word;
    ros_msg.actual_position = maxon_position_tx_msg.actual_position;
    ros_msg.actual_torque = maxon_position_tx_msg.actual_torque;
    this->maxon_position_tx_pub->publish(ros_msg);
    if (!maxon_initial_position_defined)
    {
      maxon_initial_position = maxon_position_tx_msg.actual_position;
      maxon_initial_position_defined = true;
    }
    break;
  }
  case AUTONOMOUS_TEMPORARY_MAXON_VELOCITY_TX_FRAME_ID:
  {
    autonomous_temporary_maxon_velocity_tx_t maxon_velocity_tx_msg;
    autonomous_temporary_maxon_velocity_tx_unpack(&maxon_velocity_tx_msg, frame.data, frame.can_dlc);
    lart_msgs::msg::MaxonVelocityTx ros_msg;
    ros_msg.status_word = maxon_velocity_tx_msg.status_word;
    ros_msg.actual_velocity = maxon_velocity_tx_msg.actual_velocity;
    ros_msg.pdw_duty_cicle_actual_value = maxon_velocity_tx_msg.status_word;
    this->maxon_velocity_tx_pub->publish(ros_msg);
    break;
  }
  case MAXON_HEARTBEAT_ID:
    if (frame.data[0] == 0x05)
    {
      this->maxon_activated = true;
    }
    break;
  }
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanBridge>());
  rclcpp::shutdown();
  return 0;
}

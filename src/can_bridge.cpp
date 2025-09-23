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

void CanBridge::send_can_frames(){
    while(rclcpp::ok()){
        {
            this->sendState();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
}

void CanBridge::handle_can_frame(struct can_frame frame){
    // Handle the received CAN frame
    RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: 0x%X", frame.can_id);
    // Add your handling logic here
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

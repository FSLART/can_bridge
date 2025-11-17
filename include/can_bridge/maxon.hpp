#ifndef MAXON_HPP
#define MAXON_HPP
/**
 * @file maxon.hpp
 * @author Andre Lopes (2230779@my.ipleiria.pt)
 * @brief a header file to define maxon related CAN frames and functions
 * @version 0.1
 * @date 2025-11-05
 * 
 * @copyright Copyright (c) 2025
 * 
 */


 /* Maxon activation order:
  * 1. send turnOnCANOpenDevicesFrame
  * 2. send enterPreOpModeCmdFrame
  * 3. send opModeCmdFrame
  * 4. send setVelocityCmd()
  * 5. send shutdownCmdFrame // not sure if it is needed
  * 6. send newValueFrame
  * !!Between each step there should be a delay of at least 2ms!!
 */

#include "lart_common.h"
#include "linux/can.h"
#include <cmath>
#include "CAN_DBC/generated/Autonomous_temporary/autonomous_temporary.h"

#define MAX_ACTUATOR_RELATIVE_POS 492000
#define MAXON_NODE_ID 0x08
#define PROFILE_VELOCITY 5000
#define PROFILE_ACCELERATION 7000

//this does switchOn/enable and togle the 'newPosition' bit
static const struct can_frame newValueFrame = {
    .can_id = 0x205,
    .can_dlc = 2,
    .data = { 0x0F, 0x00 }
};

static const struct can_frame turnOnCANOpenDevicesFrame = {
    .can_id = 0x00,
    .can_dlc = 2,
    .data = { 0x00, 0x00 }
};

static const struct can_frame enterPreOpModeCmdFrame = {
    .can_id = 0x00,
    .can_dlc = 2,
    .data = { 0x80, MAXON_NODE_ID }
};

static const struct can_frame opModeCmdFrame = {
    .can_id = 0x00,
    .can_dlc = 2,
    .data = { 0x01, 0x00 }
};

static const struct can_frame shutdownCmdFrame = {
    .can_id = 0x205,
    .can_dlc = 2,
    .data = { 0x06, 0x00 }
};

static const struct can_frame disableMaxonCmdFrame = {
    .can_id = 0x00,
    .can_dlc = 2,
    .data = { 0x81, MAXON_NODE_ID }
};


inline struct can_frame setVelocityCmd(){
    autonomous_temporary_maxon_profile_rx_t velocity_msg;
    velocity_msg.profile_velocity=PROFILE_VELOCITY;
    velocity_msg.profile_acceleration=PROFILE_ACCELERATION;

    struct can_frame frame;
    autonomous_temporary_maxon_profile_rx_pack(frame.data, &velocity_msg, sizeof(velocity_msg));
    frame.can_id = AUTONOMOUS_TEMPORARY_MAXON_PROFILE_RX_FRAME_ID;
    frame.can_dlc = AUTONOMOUS_TEMPORARY_MAXON_PROFILE_RX_LENGTH;

    return frame;
}

 /*
  * Before any position command is sent, you need to send the newValueFrame, and wait at least 2ms, then send the positonToMaxonCmd()
 */
inline struct can_frame positionToMaxonCmd(long offset, float angle){
    //wheel angle in rad to steering wheel angle
    float sw_angle = -61.6073*pow(angle, 4)+449.05708*pow(angle, 3)+16.71117*pow(angle, 2)+156.50789*angle;

    long raw_pos = RAD_SW_ANGLE_TO_ACTUATOR_POS(DEG_TO_RAD(sw_angle));

    if (abs(raw_pos) > MAX_ACTUATOR_RELATIVE_POS){
        raw_pos = (raw_pos > 0) ? MAX_ACTUATOR_RELATIVE_POS : -MAX_ACTUATOR_RELATIVE_POS;
    }

    long target_pos = offset + raw_pos;

    autonomous_temporary_maxon_position_rx_t position_msg;
    position_msg.control_word = 0x3F;
    position_msg.target_position = target_pos;

    struct can_frame frame;
    autonomous_temporary_maxon_position_rx_pack(frame.data, &position_msg, sizeof(position_msg));
    frame.can_id = AUTONOMOUS_TEMPORARY_MAXON_POSITION_RX_FRAME_ID;
    frame.can_dlc = AUTONOMOUS_TEMPORARY_MAXON_POSITION_RX_LENGTH;
    
    return frame;
}

#endif // MAXON_HPP
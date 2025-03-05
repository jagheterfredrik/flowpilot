#include "safety_hyundai_common.h"

int OP_LKAS_live = 0;
int OP_MDPS_live = 0;
int OP_CLU_live = 0;
int OP_SCC_live = 0;
int car_SCC_live = 0;
int OP_EMS_live = 0;
int HKG_mdps_bus = -1;
int HKG_scc_bus = -1;

bool HKG_LCAN_on_bus1 = false;
bool HKG_LCAN_on_bus2 = false;
bool HKG_forward_bus1 = false;
bool HKG_forward_obd = false;
bool HKG_forward_bus2 = true;
int HKG_LKAS_bus0_cnt = 0;
int HKG_Lcan_bus1_cnt = 0;

const CanMsg HYUNDAI_COMMUNITY_TX_MSGS[] = {
  {832, 0, 8}, {832, 2, 8}, // LKAS11 Bus 0, 2
  {1265, 0, 4}, {1265, 1, 4}, {1265, 2, 4}, // CLU11 Bus 0, 1, 2
  {1157, 0, 4}, // LFAHDA_MFC Bus 0
  {593, 0, 8}, {593, 2, 8},  // MDPS12, Bus 0, 2
  {1056, 0, 8}, //   SCC11,  Bus 0
  {1057, 0, 8}, //   SCC12,  Bus 0
  {1290, 0, 8}, //   SCC13,  Bus 0
  {905, 0, 8},  //   SCC14,  Bus 0
  {1186, 0, 8},  //   4a2SCC, Bus 0
  {790, 2, 8}, // EMS11, Bus 2
  {1155, 0, 8}, //   FCA12,  Bus 0
  {909, 0, 8},  //   FCA11,  Bus 0
  {2000, 0, 8},  // SCC_DIAG, Bus 0
  {882, 0, 8}, {882, 1, 8}, {882, 2, 8}, // ELECT_GEAR Bus 0, 1, 2
  {514, 0, 8}, {514, 1, 8}, // REGEN_LEVEL Bus 0, 1
};

// older hyundai models have less checks due to missing counters and checksums
AddrCheckStruct hyundai_community_addr_checks[] = {
  {.msg = {{608, 0, 8, .check_checksum = true, .max_counter = 3U, .expected_timestep = 10000U},
           {881, 0, 8, .expected_timestep = 10000U}, { 0 }}},
  {.msg = {{902, 0, 8, .expected_timestep = 20000U}, { 0 }, { 0 }}},
  // {.msg = {{916, 0, 8, .expected_timestep = 20000U}}}, some Santa Fe does not have this msg, need to find alternative
};

#define HYUNDAI_COMMUNITY_ADDR_CHECK_LEN (sizeof(hyundai_community_addr_checks) / sizeof(hyundai_community_addr_checks[0]))
addr_checks hyundai_community_rx_checks = {hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};

static int hyundai_community_rx_hook(CANPacket_t *to_push) {

  // Extract the address (addr) and bus number (bus) from the incoming CAN packet
  int addr = GET_ADDR(to_push);
  int bus = GET_BUS(to_push);

  // Perform a safety check on the CAN packet using pre-defined checks and checksum functions
  bool valid = addr_safety_check(to_push, &hyundai_community_rx_checks,
                            hyundai_get_checksum, hyundai_compute_checksum,
                            hyundai_get_counter, NULL);

  // If the packet is not valid, print the address of the invalid packet
  if (!valid){
    puth(addr);
  }

  // Check if LCAN is active on Bus 1; if so, invalidate the packet
  if (bus == 1 && HKG_LCAN_on_bus1) {
    valid = false;
  }

  // Check for specific addresses on Bus 1 (1296 or 524)
  if (bus == 1 && (addr == 1296 || addr == 524)) {
    // If the message is from Bus 1, reset the LCAN counter to 500
    HKG_Lcan_bus1_cnt = 500;
    // If the bus is active or forwarding, set LCAN on Bus 1 to true and disable forwarding
    if (HKG_forward_bus1 || !HKG_LCAN_on_bus1) {
      HKG_LCAN_on_bus1 = true;
      HKG_forward_bus1 = false;
    }
  }

  // Handle LKAS (Lane Keeping Assist System) messages
  if (addr == 832) {
    // If the message is on Bus 0 and forwarding is active for Bus 2, disable forwarding
    if (bus == 0 && HKG_forward_bus2) {
      HKG_forward_bus2 = false;
      HKG_LKAS_bus0_cnt = 20; // Set a counter for LKAS on Bus 0
    }
    // If the message is on Bus 2, decrement the LKAS counter and control forwarding
    if (bus == 2) {
      if (HKG_LKAS_bus0_cnt > 0) {
        HKG_LKAS_bus0_cnt--; // Decrement the counter for LKAS on Bus 0
      } else if (!HKG_forward_bus2) {
        HKG_forward_bus2 = true; // Enable forwarding if the counter reaches 0
      }
      // Decrement LCAN Bus 1 counter and turn off LCAN on Bus 1 if the counter reaches 0
      if (HKG_Lcan_bus1_cnt > 0) {
        HKG_Lcan_bus1_cnt--;
      } else if (HKG_LCAN_on_bus1) {
        HKG_LCAN_on_bus1 = false;
      }
    }
  }

  // Handle MDPS (Motor-Driven Power Steering) messages
  if ((addr == 593 || addr == 897) && HKG_mdps_bus != bus) {
    // If the bus is not Bus 2 or if specific conditions are met, update the MDPS bus
    if (bus != 2 || (!HKG_LCAN_on_bus1 || HKG_forward_obd)) {
      HKG_mdps_bus = bus;
      // If Bus 2 is selected, check for forwarding requirements
      if (bus == 2 && !HKG_forward_obd) {
        if (!HKG_forward_bus2 && !HKG_LCAN_on_bus1) {
          HKG_forward_bus2 = true;
        }
      }
    }
  }

  // Handle SCC (Smart Cruise Control) messages
  if ((addr == 1056 || addr == 1057) && HKG_scc_bus != bus) {
    // If the bus is not Bus 2 or if LCAN is not on Bus 1, update the SCC bus
    if (bus != 2 || !HKG_LCAN_on_bus2) {
      HKG_scc_bus = bus;
      // If Bus 2 is selected, ensure forwarding is active
      if (bus == 2) {
        if (!HKG_forward_bus2) {
          HKG_forward_bus2 = true;
        }
      }
    }
  }

  // If the packet is valid, process the data
  if (valid) {
    // Process torque driver data for MDPS messages on the correct bus
    if (addr == 593 && bus == HKG_mdps_bus) {
      int torque_driver_new = ((GET_BYTES(to_push, 0, 4) & 0x7ffU) * 0.79) - 808; // Scale new driver torque signal to match previous one
      // Update array of torque samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // Process SCC availability state if the car doesn’t have live SCC
    if (addr == 1056 && !OP_SCC_live) {
      int cruise_available = GET_BIT(to_push, 0U); // Check the availability of cruise control
      hyundai_common_cruise_state_check(cruise_available);
    }

    // Handle cruise control buttons for cars without live SCC
    if (addr == 1265 && bus == 0 && HKG_scc_bus == -1 && !OP_SCC_live) {
      int cruise_button = GET_BYTE(to_push, 0) & 0x7U;
      // Enable controls if "res+" or "set-" buttons are pressed
      if (!controls_allowed && (cruise_button == 1 || cruise_button == 2)) {
        controls_allowed = 1;
      }
      // Disable controls if "cancel" button is pressed
      if (cruise_button == 4) {
        controls_allowed = 0;
      }
    }

    // Sample wheel speed by averaging the speeds from opposite corners
    if (addr == 902 && bus == 0) {
      uint32_t front_left_speed = GET_BYTES(to_push, 0, 2) & 0x3FFFU;
      uint32_t rear_right_speed = GET_BYTES(to_push, 6, 2) & 0x3FFFU;
      // Determine if the vehicle is moving based on wheel speeds
      vehicle_moving = (front_left_speed > HYUNDAI_STANDSTILL_THRSLD) || (rear_right_speed > HYUNDAI_STANDSTILL_THRSLD);
    }

    // Reset brake and gas pressed states
    gas_pressed = brake_pressed = false;

    // Perform generic RX checks for LKAS on Bus 0
    generic_rx_checks((addr == 832 && bus == 0));
  }

  // Return the validity of the packet
  return valid;
}


static int hyundai_community_tx_hook(CANPacket_t *to_send) {

  int tx = 1;
  int addr = GET_ADDR(to_send);
  int bus = GET_BUS(to_send);

  tx = msg_allowed(to_send, HYUNDAI_COMMUNITY_TX_MSGS, sizeof(HYUNDAI_COMMUNITY_TX_MSGS)/sizeof(HYUNDAI_COMMUNITY_TX_MSGS[0]));

  // LKA STEER: safety check
  if (addr == 832) {
    OP_LKAS_live = 20;
    int desired_torque = ((GET_BYTES(to_send, 0, 4) >> 16) & 0x7ffU) - 1024U;
    bool steer_req = GET_BIT(to_send, 27U) != 0U;

    const SteeringLimits limits = hyundai_alt_limits ? HYUNDAI_STEERING_LIMITS_ALT : HYUNDAI_STEERING_LIMITS;
    if (steer_torque_cmd_checks(desired_torque, steer_req, limits)) {
      tx = 0;
    }
  }

  // FORCE CANCEL: safety check only relevant when spamming the cancel button.
  // ensuring that only the cancel button press is sent (VAL 4) when controls are off.
  // This avoids unintended engagements while still allowing resume spam
  //allow clu11 to be sent to MDPS if MDPS is not on bus0
  if (addr == 1265 && !controls_allowed && (bus != HKG_mdps_bus && HKG_mdps_bus == 2)) {
    if ((GET_BYTES(to_send, 0, 4) & 0x7U) != 4U) {
      tx = 0;
    }
  }

  if (addr == 593) {OP_MDPS_live = 20;}
  if (addr == 1265 && bus == 1) {OP_CLU_live = 20;} // only count mesage created for MDPS
  if (addr == 1057) {OP_SCC_live = 20; if (car_SCC_live > 0) {car_SCC_live -= 1;}}
  if (addr == 790) {OP_EMS_live = 20;}

  // 1 allows the message through
  return tx;
}

static int hyundai_community_fwd_hook(int bus_num, int addr) {

  int bus_fwd = -1; // Default: No forwarding
  
  // Forward messages between camera and CAN, except LKAS commands
  if (HKG_forward_bus2) {
    if (bus_num == 0) { // Handling messages coming from bus 0
      if (!OP_CLU_live || addr != 1265 || HKG_mdps_bus == 2) { // Cluster message condition
        if (!OP_MDPS_live || addr != 593) { // MDPS message condition
          if (!OP_EMS_live || addr != 790) { // EMS message condition
            bus_fwd = 2; // Forward to bus 2 (MDPS is on bus 2 now)
          } else {
            bus_fwd = 2;  // Special case: Forward EMS11 to bus 2
            OP_EMS_live -= 1; // Decrease EMS live counter
          }
        } else {
          bus_fwd = 0;  // Forward MDPS to bus 0 for LKAS
          OP_MDPS_live -= 1;
        }
      } else {
        bus_fwd = 2; // Special case: Forward CLU12 for MDPS
        OP_CLU_live -= 1;
      }
    }

    if (bus_num == 2) { // Handling messages from bus 2
      if (!OP_MDPS_live || addr != 593) { // MDPS message condition
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 20; // Forward general messages to bus 20
        } else {
          bus_fwd = 2;  // Special case: Forward SCC messages to bus 2
          OP_SCC_live -= 1;
        }
      } else {
        bus_fwd = 0;  // Forward MDPS for LKAS
        OP_MDPS_live -= 1;
      }
    }

    if (bus_num == 2) { // Handling messages from bus 2 (for LKAS and others)
      if (!OP_LKAS_live || (addr != 832 && addr != 1157)) { // LKAS message condition
        if (!OP_SCC_live || (addr != 1056 && addr != 1057 && addr != 1290 && addr != 905)) {
          bus_fwd = 0; // Forward other messages to bus 0
        } else {
          bus_fwd = 2;  // Special case: Forward SCC12 messages
          OP_SCC_live -= 1;
        }
      } else if (HKG_mdps_bus == 2) {
        bus_fwd = 0; // Forward LKAS and LFA for car
        OP_LKAS_live -= 1;
      } else {
        // Additional logic (truncated in original code)
      }
    }

  }
  
  return bus_fwd; // Return the determined bus forwarding value
}


static const addr_checks* hyundai_community_init(uint16_t param) {
  hyundai_common_init(param);
  controls_allowed = false;
  relay_malfunction_reset();

  // if (current_board->has_obd && HKG_forward_obd) {
  //   current_board->set_can_mode(CAN_MODE_OBD_CAN2);
  // }

  hyundai_community_rx_checks = (addr_checks){hyundai_community_addr_checks, HYUNDAI_COMMUNITY_ADDR_CHECK_LEN};
  return &hyundai_community_rx_checks;
}

const safety_hooks hyundai_community_hooks = {
  .init = hyundai_community_init,
  .rx = hyundai_community_rx_hook,
  .tx = hyundai_community_tx_hook,
  .tx_lin = nooutput_tx_lin_hook,
  .fwd = hyundai_community_fwd_hook,
};

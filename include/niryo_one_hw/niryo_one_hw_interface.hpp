#pragma once

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>   // PositionJointInterface
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>        // forward use in cpp
#include <std_srvs/Trigger.h>

#include <array>
#include <vector>
#include <string>
#include <memory>
#include <cmath>

namespace dynamixel {
  class PortHandler;
  class PacketHandler;
  class GroupSyncRead;
  class GroupSyncWrite;
}

namespace niryo_one_hw {

class NiryoOneHW : public hardware_interface::RobotHW {
public:
  explicit NiryoOneHW(ros::NodeHandle nh);
  ~NiryoOneHW();                       // defined in .cpp (after including dxl headers)

  bool init();
  void read(const ros::Duration& period);
  void write(const ros::Duration& period);

private:
  // --------- helpers / callbacks ----------
  bool home(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);

  // --------- calibration: model = sign * hardware + offset ----------
  static constexpr int NJ = 6;
  std::array<double, NJ> sign_   {{ 1, 1, 1, 1, 1, 1 }};
  std::array<double, NJ> offset_ {{ 0, 0, 0, 0, 0, 0 }};

  // --------- CAN (steppers, joints 1..3) ----------
  int         can_fd_ { -1 };
  std::string can_iface_;
  std::array<uint32_t,3> can_cmd_id_ {};
  std::array<uint32_t,3> can_stat_id_ {};
  double      can_pos_scale_ { 1e6 }, can_vel_scale_ { 1e6 };

  bool initCAN();
  void readSteppers();
  void writeSteppers();

  // --------- Dynamixel (joints 4..6) ----------
  std::string dxl_port_;
  int         dxl_baud_ { 1000000 };
  std::vector<int> dxl_ids_;

  std::unique_ptr<dynamixel::PortHandler>    dxl_porth_;
  std::unique_ptr<dynamixel::PacketHandler>  dxl_pkth_;
  std::unique_ptr<dynamixel::GroupSyncRead>  dxl_read_pos_;
  std::unique_ptr<dynamixel::GroupSyncWrite> dxl_write_goal_;

  bool initDXL();
  void readDXL();
  void writeDXL();

  // Control table (XL430 default; change if your servos differ)
  uint16_t ADDR_TORQUE_ENABLE     = 64;
  uint16_t ADDR_OPERATING_MODE    = 11;
  uint16_t ADDR_GOAL_POSITION     = 116;
  uint16_t ADDR_PRESENT_VELOCITY  = 128;
  uint16_t ADDR_PRESENT_POSITION  = 132;
  uint8_t  LEN_4                  = 4;

  double   DXL_TICKS_PER_REV = 4096.0;                     // adjust if needed
  double   DXL_RAD_TO_TICK   = DXL_TICKS_PER_REV / (2.0*M_PI);
  double   DXL_TICK_TO_RAD   = 1.0 / DXL_RAD_TO_TICK;

  // --------- ROS ----------
  ros::NodeHandle  nh_;
  ros::ServiceServer srv_home_;

  // --------- parameters ----------
  bool virtual_ { false };                 // run with virtual HAL (sim)
  std::vector<std::string> joint_names_;   // 6 joints, in URDF order
  std::vector<int>         stepper_ids_;   // joints 1..3
  // dxl_ids_ above for joints 4..6

  // --------- state/command buffers ----------
  std::array<double, NJ> pos_ {{0,0,0,0,0,0}};
  std::array<double, NJ> vel_ {{0,0,0,0,0,0}};
  std::array<double, NJ> eff_ {{0,0,0,0,0,0}};
  std::array<double, NJ> cmd_ {{0,0,0,0,0,0}};

  // --------- interfaces ----------
  hardware_interface::JointStateInterface     jnt_state_if_;
  hardware_interface::PositionJointInterface  jnt_pos_if_;
};

} // namespace niryo_one_hw

#include "niryo_one_hw/niryo_one_hw_interface.hpp"

#include <controller_manager/controller_manager.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>

// Real Dynamixel headers only in the .cpp (not in the header)
#include <dynamixel_sdk/dynamixel_sdk.h>

namespace rps = rosparam_shortcuts;
namespace niryo_one_hw {

// ---------- small helpers ----------
namespace {
inline void pack_i32(uint8_t* d, int32_t v) {
  d[0] = (v >> 24) & 0xFF; d[1] = (v >> 16) & 0xFF; d[2] = (v >> 8) & 0xFF; d[3] = v & 0xFF;
}
inline int32_t unpack_i32(const uint8_t* s) {
  return (int32_t(s[0]) << 24) | (int32_t(s[1]) << 16) | (int32_t(s[2]) << 8) | int32_t(s[3]);
}
} // anon ns

// ---------- ctor / dtor ----------
NiryoOneHW::NiryoOneHW(ros::NodeHandle nh) : nh_(nh) {}
NiryoOneHW::~NiryoOneHW() = default;

// ---------- init ----------
bool NiryoOneHW::init() {
  // nh_ is a PRIVATE handle (passed from main as "~"), so:
  //   nh_.getParam("hardware/joint_names", ...)  ->  /<ns>/<node>/hardware/joint_names
  // If you launch with ns="/niryo_robot" and name="hardware",
  // full key becomes /niryo_robot/hardware/hardware/joint_names ONLY if you mistakenly used an extra "hardware" NodeHandle layer.
  // We DO NOT create a "pnh(nh_, "hardware")" — we use nh_ directly.

  // --- Required params
  static const std::string LOG = "niryo_one_hw";
  size_t errors = 0;

  // --- Required params
  errors += !rps::get(LOG, nh_, "hardware/joint_names", joint_names_);
  nh_.param("hardware/virtual", virtual_, false);

  // CAN (scalars via rps, arrays of ints via plain getParam)
  nh_.param<std::string>("hardware/can/interface", can_iface_, std::string("can0"));
  nh_.param("hardware/can/pos_scale", can_pos_scale_, 1e6);
  nh_.param("hardware/can/vel_scale", can_vel_scale_, 1e6);

  // Integer vectors: use getParam
  std::vector<int> tmp_cmd, tmp_stat;
  if (!nh_.getParam("hardware/can/cmd_ids", tmp_cmd) || tmp_cmd.size() != 3) {
    ROS_ERROR("Missing/invalid ~hardware/can/cmd_ids (need 3 ints)");
    ++errors;
  } else {
    for (int i=0;i<3;++i) can_cmd_id_[i] = static_cast<uint32_t>(tmp_cmd[i]);
  }
  if (!nh_.getParam("hardware/can/stat_ids", tmp_stat) || tmp_stat.size() != 3) {
    ROS_ERROR("Missing/invalid ~hardware/can/stat_ids (need 3 ints)");
    ++errors;
  } else {
    for (int i=0;i<3;++i) can_stat_id_[i] = static_cast<uint32_t>(tmp_stat[i]);
  }

  // DXL comm settings
  nh_.param<std::string>("hardware/dxl/port", dxl_port_, std::string("/dev/ttyUSB0"));
  nh_.param("hardware/dxl/baud", dxl_baud_, 1000000);

  // More integer vectors: use getParam
  if (!nh_.getParam("hardware/stepper/ids", stepper_ids_) || stepper_ids_.size() != 3) {
    ROS_ERROR("Missing/invalid ~hardware/stepper/ids (need 3 ints)");
    ++errors;
  }
  if (!nh_.getParam("hardware/dxl/ids", dxl_ids_) || dxl_ids_.size() != 3) {
    ROS_ERROR("Missing/invalid ~hardware/dxl/ids (need 3 ints)");
    ++errors;
  }

  if (joint_names_.size() != NJ) {
    ROS_ERROR_STREAM("Expected " << NJ << " joint names, got " << joint_names_.size());
    ++errors;
  }

  // --- Register joint interfaces
  for (int i = 0; i < NJ; ++i) {
    auto sh = hardware_interface::JointStateHandle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_if_.registerHandle(sh);
    auto jh = hardware_interface::JointHandle(sh, &cmd_[i]);
    jnt_pos_if_.registerHandle(jh);
  }
  registerInterface(&jnt_state_if_);
  registerInterface(&jnt_pos_if_);

  // --- Calibration (read from global tree)
  {
    ros::NodeHandle ng; // global
    for (int i = 0; i < NJ; ++i) {
      const std::string jn = "joint_" + std::to_string(i + 1);
      ng.param("/niryo_robot/calibration/" + jn + "/sign",       sign_[i],   1.0);
      ng.param("/niryo_robot/calibration/" + jn + "/offset_rad", offset_[i], 0.0);
    }
  }

  // --- Init buses only in REAL mode
  if (!virtual_) {
    if (!initCAN()) { ROS_ERROR("initCAN failed"); ++errors; }
    if (!initDXL()) { ROS_ERROR("initDXL failed"); ++errors; }
    ROS_INFO_STREAM("HW mode: REAL | CAN=" << can_iface_ << " | DXL=" << dxl_port_ << "@" << dxl_baud_);
  } else {
    ROS_WARN("HW mode: VIRTUAL (no robot connected)");
  }

  // Homing service (private relative to nh_)
  srv_home_ = nh_.advertiseService("home", &NiryoOneHW::home, this);

  // NOTE: shutdownIfError signature is (LOG, error_count)
  rps::shutdownIfError(LOG, errors);
  if (errors) return false;

  ROS_INFO("niryo_one_hw: init OK");
  return true;
}

// ---------- read/write ----------
void NiryoOneHW::read(const ros::Duration& period) {
  (void)period;
  if (!virtual_) {
    readSteppers();
    readDXL();
  }
}

void NiryoOneHW::write(const ros::Duration& period) {
  if (virtual_) {
    const double dt = period.toSec();
    const double a  = std::max(0.0, std::min(1.0, dt * 5.0)); // smooth converge
    for (int i = 0; i < NJ; ++i) {
      const double p0 = pos_[i];
      pos_[i] = p0 + a * (cmd_[i] - p0);
      vel_[i] = (pos_[i] - p0) / std::max(1e-6, dt);
      eff_[i] = 0.0;
    }
    return;
  }
  (void)period;
  writeSteppers();
  writeDXL();
}

// ---------- services ----------
bool NiryoOneHW::home(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
  for (int i = 0; i < NJ; ++i) cmd_[i] = pos_[i];
  res.success = true;
  res.message = virtual_ ? "Homing skipped (virtual)" : "Homing stub";
  return true;
}

// ---------- CAN ----------
bool NiryoOneHW::initCAN() {
  can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (can_fd_ < 0) { ROS_ERROR("socketCAN open failed"); return false; }

  ifreq ifr{}; std::strncpy(ifr.ifr_name, can_iface_.c_str(), IFNAMSIZ - 1);
  if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) { ROS_ERROR("SIOCGIFINDEX failed"); return false; }

  sockaddr_can addr{}; addr.can_family = AF_CAN; addr.can_ifindex = ifr.ifr_ifindex;
  // non-blocking
  int flags = fcntl(can_fd_, F_GETFL, 0);
  fcntl(can_fd_, F_SETFL, flags | O_NONBLOCK);
  if (bind(can_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
    ROS_ERROR("bind(can) failed"); return false;
  }
  return true;
}

void NiryoOneHW::writeSteppers() {
  for (int j = 0; j < 3; ++j) {
    can_frame f{}; f.can_id = can_cmd_id_[j]; f.can_dlc = 6;
    f.data[0] = 0x01; // example opcode
    const double  q_hw = (cmd_[j] - offset_[j]) * sign_[j];  // invert mapping: model -> hw
    const int32_t q_fp = static_cast<int32_t>(std::lround(q_hw * can_pos_scale_));
    pack_i32(&f.data[1], q_fp);
    const ssize_t n = ::write(can_fd_, &f, sizeof(f));
    if (n != static_cast<ssize_t>(sizeof(f))) ROS_WARN_THROTTLE(1.0, "CAN TX failed joint %d", j);
  }
}

void NiryoOneHW::readSteppers() {
  can_frame f{};
  while (::read(can_fd_, &f, sizeof(f)) == static_cast<ssize_t>(sizeof(f))) {
    int j = -1;
    for (int k = 0; k < 3; ++k) if (f.can_id == can_stat_id_[k]) { j = k; break; }
    if (j < 0 || f.can_dlc < 5) continue;

    if (f.data[0] == 0x10) { // present position
      const double q_hw = double(unpack_i32(&f.data[1])) / can_pos_scale_;
      pos_[j] = sign_[j] * q_hw + offset_[j];                // hw -> model
    } else if (f.data[0] == 0x11) { // present velocity
      const double v_hw = double(unpack_i32(&f.data[1])) / can_vel_scale_;
      vel_[j] = sign_[j] * v_hw;                              // sign only
    }
  }
}

// ---------- DXL ----------
bool NiryoOneHW::initDXL() {
  dxl_porth_.reset(dynamixel::PortHandler::getPortHandler(dxl_port_.c_str()));
  dxl_pkth_.reset(dynamixel::PacketHandler::getPacketHandler(2.0));
  if (!dxl_porth_->openPort())      { ROS_ERROR("DXL: openPort failed"); return false; }
  if (!dxl_porth_->setBaudRate(dxl_baud_)) { ROS_ERROR("DXL: setBaudRate failed"); return false; }

  // set position mode + torque on
  for (int id : dxl_ids_) {
    int dxl_comm_result; uint8_t dxl_error = 0;
    dxl_comm_result = dxl_pkth_->write1ByteTxRx(dxl_porth_.get(), id, ADDR_OPERATING_MODE, 3, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error) ROS_WARN("DXL %d: set mode err=%d", id, dxl_error);
    dxl_comm_result = dxl_pkth_->write1ByteTxRx(dxl_porth_.get(), id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS || dxl_error) ROS_WARN("DXL %d: torque on err=%d", id, dxl_error);
  }

  dxl_read_pos_.reset(new dynamixel::GroupSyncRead (dxl_porth_.get(), dxl_pkth_.get(), ADDR_PRESENT_POSITION, LEN_4));
  for (int id : dxl_ids_) dxl_read_pos_->addParam(id);
  dxl_write_goal_.reset(new dynamixel::GroupSyncWrite(dxl_porth_.get(), dxl_pkth_.get(), ADDR_GOAL_POSITION, LEN_4));
  return true;
}

void NiryoOneHW::readDXL() {
  if (!dxl_read_pos_) return;
  if (dxl_read_pos_->txRxPacket() != COMM_SUCCESS) return;

  // Map dxl_ids_[0..2] -> joints 3..5 (0-based → joints 4..6)
  for (size_t k = 0; k < dxl_ids_.size(); ++k) {
    const int id = dxl_ids_[k];
    if (!dxl_read_pos_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_4)) continue;
    const uint32_t raw = dxl_read_pos_->getData(id, ADDR_PRESENT_POSITION, LEN_4);
    const double   q_hw = int32_t(raw) * DXL_TICK_TO_RAD;
    const int      j    = 3 + static_cast<int>(k);
    pos_[j] = sign_[j] * q_hw + offset_[j];   // hw -> model
  }
}

void NiryoOneHW::writeDXL() {
  if (!dxl_write_goal_) return;
  for (size_t k = 0; k < dxl_ids_.size(); ++k) {
    const int id = dxl_ids_[k];
    const int j  = 3 + static_cast<int>(k);
    const double  q_hw = (cmd_[j] - offset_[j]) * sign_[j];   // model -> hw
    const int32_t goal = static_cast<int32_t>(std::lround(q_hw * DXL_RAD_TO_TICK));
    uint8_t param[4] = {
      uint8_t(goal & 0xFF), uint8_t((goal >> 8) & 0xFF),
      uint8_t((goal >> 16) & 0xFF), uint8_t((goal >> 24) & 0xFF)
    };
    dxl_write_goal_->addParam(id, param);
  }
  dxl_write_goal_->txPacket();
  dxl_write_goal_->clearParam();
}

} // namespace niryo_one_hw

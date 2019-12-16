#ifndef HARDWARE_INTERFACE_EXTENSIONS_POSVELEFF_COMMAND_INTERFACE_HPP
#define HARDWARE_INTERFACE_EXTENSIONS_POSVELEFF_COMMAND_INTERFACE_HPP

#include <cassert>
#include <string>

#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>

namespace hardware_interface_extensions {

class PosVelEffJointHandle : public hardware_interface::PosVelJointHandle {
public:
  PosVelEffJointHandle() : hardware_interface::PosVelJointHandle(), cmd_eff_(NULL) {}

  PosVelEffJointHandle(const hardware_interface::JointStateHandle &js, double *cmd_pos,
                       double *cmd_vel, double *cmd_eff)
      : hardware_interface::PosVelJointHandle(js, cmd_pos, cmd_vel), cmd_eff_(cmd_eff) {
    if (!cmd_eff) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create handle '" + js.getName() + "'. Command effort data pointer is null.");
    }
  }

  void setCommand(const double cmd_pos, const double cmd_vel, const double cmd_eff) {
    setCommandPosition(cmd_pos);
    setCommandVelocity(cmd_vel);
    setCommandEffort(cmd_eff);
  }

  void setCommandEffort(const double cmd_eff) {
    assert(cmd_eff_);
    *cmd_eff_ = cmd_eff;
  }

  double getCommandEffort() const {
    assert(cmd_eff_);
    return *cmd_eff_;
  }

private:
  double *cmd_eff_;
};

class PosVelEffJointInterface
    : public hardware_interface::HardwareResourceManager< PosVelEffJointHandle,
                                                          hardware_interface::ClaimResources > {};

} // namespace hardware_interface_extensions

#endif

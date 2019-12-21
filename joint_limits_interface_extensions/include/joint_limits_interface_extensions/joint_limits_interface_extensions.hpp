#ifndef JOINT_LIMITS_INTERFACE_EXTENSIONS_JOINT_LIMITS_INTERFACE_EXTENSIONS_HPP
#define JOINT_LIMITS_INTERFACE_EXTENSIONS_JOINT_LIMITS_INTERFACE_EXTENSIONS_HPP

#include <string>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface_extensions/posveleff_command_interface.hpp>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface_extensions/common_namespaces.hpp>
#include <ros/duration.h>

#include <boost/foreach.hpp>

namespace joint_limits_interface_extensions {

class PosVelJointSaturationHandle {
public:
  PosVelJointSaturationHandle(const hi::PosVelJointHandle &posvel_handle,
                              const jli::JointLimits &limits)
      : posvel_handle_(posvel_handle),
        pos_sat_handle_(makeJointHandle(posvel_handle.getName(), &pos_, &vel_, &eff_, &pos_cmd_),
                        limits),
        vel_sat_handle_(makeJointHandle(posvel_handle.getName(), &pos_, &vel_, &eff_, &vel_cmd_),
                        limits) {}

  std::string getName() const { return posvel_handle_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    // read local data from joint data
    pos_ = posvel_handle_.getPosition();
    vel_ = posvel_handle_.getVelocity();
    eff_ = posvel_handle_.getEffort();
    pos_cmd_ = posvel_handle_.getCommandPosition();
    vel_cmd_ = posvel_handle_.getCommandVelocity();

    // saturate local commands
    pos_sat_handle_.enforceLimits(period);
    vel_sat_handle_.enforceLimits(period);

    // update joint commands with local commands
    posvel_handle_.setCommandPosition(pos_cmd_);
    posvel_handle_.setCommandVelocity(vel_cmd_);
  }

  void reset() { pos_sat_handle_.reset(); }

protected:
  static hi::JointHandle makeJointHandle(const std::string &name, double *pos, double *vel,
                                         double *eff, double *cmd) {
    return hi::JointHandle(hi::JointStateHandle(name, pos, vel, eff), cmd);
  }

protected:
  double pos_, vel_, eff_, pos_cmd_, vel_cmd_;
  hi::PosVelJointHandle posvel_handle_;
  jli::PositionJointSaturationHandle pos_sat_handle_;
  jli::VelocityJointSaturationHandle vel_sat_handle_;
};

class PosVelEffJointSaturationHandle : public PosVelJointSaturationHandle {
public:
  PosVelEffJointSaturationHandle(const hie::PosVelEffJointHandle &posveleff_handle,
                                 const jli::JointLimits &limits)
      : PosVelJointSaturationHandle(posveleff_handle, limits), posveleff_handle_(posveleff_handle),
        eff_sat_handle_(makeJointHandle(posveleff_handle.getName(), &pos_, &vel_, &eff_, &eff_cmd_),
                        limits) {}

  std::string getName() const { return posveleff_handle_.getName(); }

  void enforceLimits(const ros::Duration &period) {
    // read local data from joint data
    pos_ = posveleff_handle_.getPosition();
    vel_ = posveleff_handle_.getVelocity();
    eff_ = posveleff_handle_.getEffort();
    pos_cmd_ = posveleff_handle_.getCommandPosition();
    vel_cmd_ = posveleff_handle_.getCommandVelocity();
    eff_cmd_ = posveleff_handle_.getCommandEffort();

    // saturate local commands
    pos_sat_handle_.enforceLimits(period);
    vel_sat_handle_.enforceLimits(period);
    eff_sat_handle_.enforceLimits(period);

    // update joint commands with local commands
    posveleff_handle_.setCommandPosition(pos_cmd_);
    posveleff_handle_.setCommandVelocity(vel_cmd_);
    posveleff_handle_.setCommandEffort(eff_cmd_);
  }

  void reset() { pos_sat_handle_.reset(); }

protected:
  double eff_cmd_;
  hie::PosVelEffJointHandle posveleff_handle_;
  jli::EffortJointSaturationHandle eff_sat_handle_;
};

class PosVelJointSaturationInterface
    : public jli::JointLimitsInterface< PosVelJointSaturationHandle > {
public:
  void reset() {
    BOOST_FOREACH (ResourceMap::value_type &handle, resource_map_) { handle.second.reset(); }
  }
};

class PosVelEffJointSaturationInterface
    : public jli::JointLimitsInterface< PosVelEffJointSaturationHandle > {
public:
  void reset() {
    BOOST_FOREACH (ResourceMap::value_type &handle, resource_map_) { handle.second.reset(); }
  }
};
} // namespace joint_limits_interface_extensions

#endif
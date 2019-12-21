#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_POSVEL_JOINT_INTERFACE_PROVIDER_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_POSVEL_JOINT_INTERFACE_PROVIDER_HPP

#include <string>
#include <vector>

#include <hardware_interface/actuator_command_interface.h>
#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
// for RequisiteProvider, JointInterfaces, RawJointDataMap, JointData
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface_extensions/common_namespaces.hpp>
#include <transmission_interface_extensions/joint_state_interface_provider.hpp>

#include <boost/foreach.hpp>

namespace transmission_interface_extensions {

class PosVelJointInterfaceProvider : public JointStateInterfaceProvider {
public:
  PosVelJointInterfaceProvider() {}

  virtual ~PosVelJointInterfaceProvider() {}

  virtual bool updateJointInterfaces(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                     ti::JointInterfaces &jnt_ifaces,
                                     ti::RawJointDataMap &raw_jnt_data_map) {
    // register state interface & handles if never
    if (!JointStateInterfaceProvider::updateJointInterfaces(trans_info, robot_hw, jnt_ifaces,
                                                            raw_jnt_data_map)) {
      return false;
    }

    // register posvel interface if never
    if (!robot_hw->get< hi::PosVelJointInterface >()) {
      robot_hw->registerInterface(&getPosVelJointInterface());
    }
    hi::PosVelJointInterface &iface(*robot_hw->get< hi::PosVelJointInterface >());

    // register posvel handle to the posvel interface if never
    BOOST_FOREACH (const ti::JointInfo &jnt_info, trans_info.joints_) {
      if (!hasResource(jnt_info.name_, iface)) {
        ti::RawJointData &raw_jnt_data(raw_jnt_data_map[jnt_info.name_]);
        iface.registerHandle(
            hi::PosVelJointHandle(jnt_ifaces.joint_state_interface.getHandle(jnt_info.name_),
                                  &raw_jnt_data.position_cmd, &raw_jnt_data.velocity_cmd));
      }
    }

    return true;
  }

protected:
  virtual bool getJointCommandData(const ti::TransmissionInfo &trans_info,
                                   const ti::RawJointDataMap &raw_jnt_data_map,
                                   ti::JointData &jnt_cmd_data) {
    // allocate destination joint command data
    const std::size_t dim(trans_info.joints_.size());
    jnt_cmd_data.position.resize(dim);
    jnt_cmd_data.velocity.resize(dim);

    // map source to destination joint command data
    for (std::size_t i = 0; i < dim; ++i) {
      // find source data
      const ti::RawJointDataMap::const_iterator raw_jnt_data(
          raw_jnt_data_map.find(trans_info.joints_[i].name_));
      if (raw_jnt_data == raw_jnt_data_map.end()) {
        return false;
      }

      // map data
      jnt_cmd_data.position[i] = const_cast< double * >(&raw_jnt_data->second.position_cmd);
      jnt_cmd_data.velocity[i] = const_cast< double * >(&raw_jnt_data->second.velocity_cmd);
    }

    return true;
  }

  virtual bool getActuatorCommandData(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                      ti::ActuatorData &act_cmd_data) {
    // find source actuator command data
    std::vector< hi::ActuatorHandle > act_pos_handles, act_vel_handles;
    if (!getActuatorHandles< hi::PositionActuatorInterface >(trans_info.actuators_, robot_hw,
                                                             act_pos_handles) ||
        !getActuatorHandles< hi::VelocityActuatorInterface >(trans_info.actuators_, robot_hw,
                                                             act_vel_handles)) {
      return false;
    }

    // allocate destination actuator command data
    const std::size_t dim(trans_info.actuators_.size());
    act_cmd_data.position.resize(dim);
    act_cmd_data.velocity.resize(dim);

    // map source to destination actuator command data
    for (std::size_t i = 0; i < dim; ++i) {
      act_cmd_data.position[i] = const_cast< double * >(act_pos_handles[i].getCommandPtr());
      act_cmd_data.velocity[i] = const_cast< double * >(act_vel_handles[i].getCommandPtr());
    }

    return true;
  }

  virtual bool registerTransmission(ti::TransmissionLoaderData &loader_data,
                                    ti::RequisiteProvider::TransmissionHandleData &handle_data) {
    // register state transmission interface & handle if never
    if (!JointStateInterfaceProvider::registerTransmission(loader_data, handle_data)) {
      return false;
    }

    // quick access to the transmission manager
    ti::RobotTransmissions &trans_man(*loader_data.robot_transmissions);

    {
      // register position command transmission interface if never
      if (!trans_man.get< ti::JointToActuatorPositionInterface >()) {
        trans_man.registerInterface(&loader_data.transmission_interfaces.jnt_to_act_pos_cmd);
      }
      ti::JointToActuatorPositionInterface &pos_cmd_iface(
          *trans_man.get< ti::JointToActuatorPositionInterface >());

      // register position command transmission handle if never
      if (!hasResource(handle_data.name, pos_cmd_iface)) {
        pos_cmd_iface.registerHandle(
            ti::JointToActuatorPositionHandle(handle_data.name, handle_data.transmission.get(),
                                              handle_data.act_cmd_data, handle_data.jnt_cmd_data));
      }
    }

    {
      // register velocity command transmission interface if never
      if (!trans_man.get< ti::JointToActuatorVelocityInterface >()) {
        trans_man.registerInterface(&loader_data.transmission_interfaces.jnt_to_act_vel_cmd);
      }
      ti::JointToActuatorVelocityInterface &vel_cmd_iface(
          *trans_man.get< ti::JointToActuatorVelocityInterface >());

      // register velocity command transmission handle if never
      if (!hasResource(handle_data.name, vel_cmd_iface)) {
        vel_cmd_iface.registerHandle(
            ti::JointToActuatorVelocityHandle(handle_data.name, handle_data.transmission.get(),
                                              handle_data.act_cmd_data, handle_data.jnt_cmd_data));
      }
    }

    return true;
  }

private:
  // return the joint interface. it must be in the static memory space
  // to allow access from outside of this plugin.
  static hi::PosVelJointInterface &getPosVelJointInterface() {
    static hi::PosVelJointInterface iface;
    return iface;
  }
};
} // namespace transmission_interface_extensions

#endif
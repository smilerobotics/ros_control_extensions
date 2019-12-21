#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_POSVELEFF_JOINT_INTERFACE_PROVIDER_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_POSVELEFF_JOINT_INTERFACE_PROVIDER_HPP

#include <string>
#include <vector>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface_extensions/posveleff_command_interface.hpp>
#include <transmission_interface/transmission_info.h>
// for RequisiteProvider, JointInterfaces, RawJointDataMap, JointData
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface_extensions/common_namespaces.hpp>
#include <transmission_interface_extensions/posvel_joint_interface_provider.hpp>

#include <boost/foreach.hpp>

namespace transmission_interface_extensions {

class PosVelEffJointInterfaceProvider : public PosVelJointInterfaceProvider {
public:
  PosVelEffJointInterfaceProvider() {}

  virtual ~PosVelEffJointInterfaceProvider() {}

  virtual bool updateJointInterfaces(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                     ti::JointInterfaces &jnt_ifaces,
                                     ti::RawJointDataMap &raw_jnt_data_map) {
    // register state interface & handles if never
    if (!JointStateInterfaceProvider::updateJointInterfaces(trans_info, robot_hw, jnt_ifaces,
                                                            raw_jnt_data_map)) {
      return false;
    }

    // register posveleff interface if never
    if (!robot_hw->get< hie::PosVelEffJointInterface >()) {
      robot_hw->registerInterface(&getPosVelEffJointInterface());
    }
    hie::PosVelEffJointInterface &iface(*robot_hw->get< hie::PosVelEffJointInterface >());

    // register posveleff handle to the posvel interface if never
    BOOST_FOREACH (const ti::JointInfo &jnt_info, trans_info.joints_) {
      if (!hasResource(jnt_info.name_, iface)) {
        ti::RawJointData &raw_jnt_data(raw_jnt_data_map[jnt_info.name_]);
        iface.registerHandle(hie::PosVelEffJointHandle(
            jnt_ifaces.joint_state_interface.getHandle(jnt_info.name_), &raw_jnt_data.position_cmd,
            &raw_jnt_data.velocity_cmd, &raw_jnt_data.effort_cmd));
      }
    }

    return true;
  }

protected:
  virtual bool getJointCommandData(const ti::TransmissionInfo &trans_info,
                                   const ti::RawJointDataMap &raw_jnt_data_map,
                                   ti::JointData &jnt_cmd_data) {
    // get position & velocity commands
    if (!PosVelJointInterfaceProvider::getJointCommandData(trans_info, raw_jnt_data_map,
                                                           jnt_cmd_data)) {
      return false;
    }

    // allocate destination effort command data
    const std::size_t dim(trans_info.joints_.size());
    jnt_cmd_data.effort.resize(dim);

    // map source to destination effort command data
    for (std::size_t i = 0; i < dim; ++i) {
      // find source data
      const ti::RawJointDataMap::const_iterator raw_jnt_data(
          raw_jnt_data_map.find(trans_info.joints_[i].name_));
      if (raw_jnt_data == raw_jnt_data_map.end()) {
        return false;
      }

      // map data
      jnt_cmd_data.effort[i] = const_cast< double * >(&raw_jnt_data->second.effort_cmd);
    }
  }

  virtual bool getActuatorCommandData(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                      ti::ActuatorData &act_cmd_data) {
    // get position & velocity commands
    if (!PosVelJointInterfaceProvider::getActuatorCommandData(trans_info, robot_hw, act_cmd_data)) {
      return false;
    }

    // find source effort command data
    std::vector< hi::ActuatorHandle > act_eff_handles;
    if (!getActuatorHandles< hi::EffortActuatorInterface >(trans_info.actuators_, robot_hw,
                                                           act_eff_handles)) {
      return false;
    }

    // allocate destination actuator command data
    const std::size_t dim(trans_info.actuators_.size());
    act_cmd_data.effort.resize(dim);

    // map source to destination actuator command data
    for (std::size_t i = 0; i < dim; ++i) {
      act_cmd_data.effort[i] = const_cast< double * >(act_eff_handles[i].getCommandPtr());
    }

    return true;
  }

  virtual bool registerTransmission(ti::TransmissionLoaderData &loader_data,
                                    ti::RequisiteProvider::TransmissionHandleData &handle_data) {
    // register state, position & velocity transmissions
    if (!PosVelJointInterfaceProvider::registerTransmission(loader_data, handle_data)) {
      return false;
    }

    // quick access to the transmission manager
    ti::RobotTransmissions &trans_man(*loader_data.robot_transmissions);

    // register effort command transmission interface if never
    if (!trans_man.get< ti::JointToActuatorEffortInterface >()) {
      trans_man.registerInterface(&loader_data.transmission_interfaces.jnt_to_act_eff_cmd);
    }
    ti::JointToActuatorEffortInterface &iface(
        *trans_man.get< ti::JointToActuatorEffortInterface >());

    // register effort command transmission handle if never
    if (!hasResource(handle_data.name, iface)) {
      iface.registerHandle(
          ti::JointToActuatorEffortHandle(handle_data.name, handle_data.transmission.get(),
                                          handle_data.act_cmd_data, handle_data.jnt_cmd_data));
    }

    return true;
  }

private:
  // return the joint interface. it must be in the static memory space
  // to allow access from outside of this plugin.
  static hie::PosVelEffJointInterface &getPosVelEffJointInterface() {
    static hie::PosVelEffJointInterface interface;
    return interface;
  }
};
} // namespace transmission_interface_extensions

#endif
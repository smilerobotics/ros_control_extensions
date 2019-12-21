#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_JOINT_STATE_INTERFACE_PROVIDER_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_JOINT_STATE_INTERFACE_PROVIDER_HPP

#include <string>
#include <vector>

#include <hardware_interface/actuator_state_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/robot_transmissions.h>
#include <transmission_interface/transmission_info.h>
// for RequisiteProvider, JointInterfaces, RawJointDataMap, JointData
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface_extensions/common_namespaces.hpp>

#include <boost/foreach.hpp>

namespace transmission_interface_extensions {

// Equivalent of transmission_interface::JointStateInterfaceProvider (with minor bugfix).
// This is required because we cannot link the original to our plugin
// due to a limitation of the class_loader pkg. See the following page for details.
// http://wiki.ros.org/class_loader#Caution_of_Linking_Directly_Against_Plugin_Libraries
class JointStateInterfaceProvider : public ti::RequisiteProvider {
public:
  JointStateInterfaceProvider() {}

  virtual ~JointStateInterfaceProvider() {}

  virtual bool updateJointInterfaces(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                     ti::JointInterfaces &jnt_ifaces,
                                     ti::RawJointDataMap &raw_jnt_data_map) {
    // register state interface if never
    if (!robot_hw->get< hi::JointStateInterface >()) {
      robot_hw->registerInterface(&jnt_ifaces.joint_state_interface);
    }
    hi::JointStateInterface &iface(*robot_hw->get< hi::JointStateInterface >());

    // register state handle to the state interface if never
    BOOST_FOREACH (const ti::JointInfo &jnt_info, trans_info.joints_) {
      if (!hasResource(jnt_info.name_, iface)) {
        ti::RawJointData &raw_jnt_data(raw_jnt_data_map[jnt_info.name_]);
        iface.registerHandle(hi::JointStateHandle(jnt_info.name_, &raw_jnt_data.position,
                                                  &raw_jnt_data.velocity, &raw_jnt_data.effort));
      }
    }

    return true;
  }

protected:
  virtual bool getJointStateData(const ti::TransmissionInfo &trans_info,
                                 const ti::RawJointDataMap &raw_jnt_data_map,
                                 ti::JointData &jnt_state_data) {
    // allocate destination joint state data
    const std::size_t dim(trans_info.joints_.size());
    jnt_state_data.position.resize(dim);
    jnt_state_data.velocity.resize(dim);
    jnt_state_data.effort.resize(dim);

    // map source to destination joint state data
    for (std::size_t i = 0; i < dim; ++i) {
      // find source data
      const ti::RawJointDataMap::const_iterator raw_jnt_data(
          raw_jnt_data_map.find(trans_info.joints_[i].name_));
      if (raw_jnt_data == raw_jnt_data_map.end()) {
        return false;
      }

      // map data
      jnt_state_data.position[i] = const_cast< double * >(&raw_jnt_data->second.position);
      jnt_state_data.velocity[i] = const_cast< double * >(&raw_jnt_data->second.velocity);
      jnt_state_data.effort[i] = const_cast< double * >(&raw_jnt_data->second.effort);
    }

    return true;
  }

  virtual bool getActuatorStateData(const ti::TransmissionInfo &trans_info, hi::RobotHW *robot_hw,
                                    ti::ActuatorData &act_state_data) {
    // find source actuator state data
    std::vector< hi::ActuatorStateHandle > act_handles;
    if (!getActuatorHandles< hi::ActuatorStateInterface >(trans_info.actuators_, robot_hw,
                                                          act_handles)) {
      return false;
    }

    // allocate destination actuator state data
    const std::size_t dim(trans_info.actuators_.size());
    act_state_data.position.resize(dim);
    act_state_data.velocity.resize(dim);
    act_state_data.effort.resize(dim);

    // map source to destination actuator state data
    for (std::size_t i = 0; i < dim; ++i) {
      act_state_data.position[i] = const_cast< double * >(act_handles[i].getPositionPtr());
      act_state_data.velocity[i] = const_cast< double * >(act_handles[i].getVelocityPtr());
      act_state_data.effort[i] = const_cast< double * >(act_handles[i].getEffortPtr());
    }

    return true;
  }

  virtual bool registerTransmission(ti::TransmissionLoaderData &loader_data,
                                    ti::RequisiteProvider::TransmissionHandleData &handle_data) {
    // quick access to the transmission manager
    ti::RobotTransmissions &trans_man(*loader_data.robot_transmissions);

    // register state transmission interface if never
    if (!trans_man.get< ti::ActuatorToJointStateInterface >()) {
      trans_man.registerInterface(&loader_data.transmission_interfaces.act_to_jnt_state);
    }
    ti::ActuatorToJointStateInterface &iface(*trans_man.get< ti::ActuatorToJointStateInterface >());

    // register state transmission handle if never
    if (!hasResource(handle_data.name, iface)) {
      iface.registerHandle(
          ti::ActuatorToJointStateHandle(handle_data.name, handle_data.transmission.get(),
                                         handle_data.act_state_data, handle_data.jnt_state_data));
    }

    return true;
  }
};
} // namespace transmission_interface_extensions

#endif
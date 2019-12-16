#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_PUBLIC_ADAPTER_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_PUBLIC_ADAPTER_HPP

#include <hardware_interface/robot_hw.h>
#include <transmission_interface/transmission_info.h>
#include <transmission_interface_extensions/common_namespaces.hpp>
// for JointInterfaces, RawJointDataMap, JointData
#include <transmission_interface/transmission_interface_loader.h>
#include <transmission_interface_extensions/common_namespaces.hpp>

namespace transmission_interface_extensions {

// we need to invoke protected methods of backend providers
// in PosVel or PosVelEffJointInterfaceProvider
// but cannot inherit all providers at once because of the diamond inheritance problem.
// this class allows to access these protected methods without inheritance
// by "pulling up" method permissions to public.
template < class Provider > class PublicAdapter : public Provider {
public:
  bool getJointStateData(const ti::TransmissionInfo &transmission_info,
                         const ti::RawJointDataMap &raw_joint_data_map,
                         ti::JointData &jnt_state_data) {
    return Provider::getJointStateData(transmission_info, raw_joint_data_map, jnt_state_data);
  }

  bool getJointCommandData(const ti::TransmissionInfo &transmission_info,
                           const ti::RawJointDataMap &raw_joint_data_map,
                           ti::JointData &jnt_cmd_data) {
    return Provider::getJointCommandData(transmission_info, raw_joint_data_map, jnt_cmd_data);
  }

  bool getActuatorStateData(const ti::TransmissionInfo &transmission_info, hi::RobotHW *robot_hw,
                            ti::ActuatorData &act_state_data) {
    return Provider::getActuatorStateData(transmission_info, robot_hw, act_state_data);
  }

  bool getActuatorCommandData(const ti::TransmissionInfo &transmission_info, hi::RobotHW *robot_hw,
                              ti::ActuatorData &act_cmd_data) {
    return Provider::getActuatorCommandData(transmission_info, robot_hw, act_cmd_data);
  }

  bool registerTransmission(ti::TransmissionLoaderData &loader_data,
                            ti::RequisiteProvider::TransmissionHandleData &handle_data) {
    return Provider::registerTransmission(loader_data, handle_data);
  }
};
} // namespace transmission_interface_extensions

#endif
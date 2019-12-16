#include <pluginlib/class_list_macros.h>
#include <transmission_interface/transmission_interface_loader.h> // for RequisiteProvider
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface_extensions/posvel_joint_interface_provider.hpp>
#include <transmission_interface_extensions/posveleff_joint_interface_provider.hpp>
#include <transmission_interface_extensions/slider_crank_transmission_loader.hpp>

PLUGINLIB_EXPORT_CLASS(transmission_interface_extensions::PosVelJointInterfaceProvider,
                       transmission_interface::RequisiteProvider);
PLUGINLIB_EXPORT_CLASS(transmission_interface_extensions::PosVelEffJointInterfaceProvider,
                       transmission_interface::RequisiteProvider);
PLUGINLIB_EXPORT_CLASS(transmission_interface_extensions::SliderCrankTransmissionLoader,
                       transmission_interface::TransmissionLoader);
#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_SLIDER_CRANK_TRANSMISSION_LOADER_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_SLIDER_CRANK_TRANSMISSION_LOADER_HPP

#include <stdexcept>
#include <string>
#include <vector>

#include <ros/console.h>
#include <transmission_interface/transmission_interface_exception.h>
#include <transmission_interface/transmission_loader.h>
#include <transmission_interface_extensions/common_namespaces.hpp>
#include <transmission_interface_extensions/slider_crank_transmission.hpp>

#include <boost/lexical_cast.hpp>

namespace transmission_interface_extensions {

class SliderCrankTransmissionLoader : public ti::TransmissionLoader {
public:
  virtual ti::TransmissionSharedPtr load(const ti::TransmissionInfo &src_info) {
    try {
      SliderCrankTransmissionInfo dst_info;
      parseActuator(src_info, &dst_info);
      parseJoints(src_info, &dst_info);
      return ti::TransmissionSharedPtr(new SliderCrankTransmission(dst_info));
    } catch (const std::exception &ex) {
      ROS_ERROR_STREAM("Exception caught on loading SliderCrankTransmission named "
                       << src_info.name_ << ": " << ex.what());
      return ti::TransmissionSharedPtr();
    }
  }

private:
  //
  // helper functions to parse xml
  //

  // raw values in a xml element

  static double parseOptionalValue(const TiXmlElement &xml, const std::string &child_name,
                                   const double default_value) {
    const TiXmlElement *const child(xml.FirstChildElement(child_name));
    return child ? boost::lexical_cast< double >(child->GetText()) : default_value;
  }

  static double parseRequiredValue(const TiXmlElement &xml, const std::string &child_name) {
    const TiXmlElement *const child(xml.FirstChildElement(child_name));
    if (!child) {
      throw ti::TransmissionInterfaceException("Missing required element <" + child_name + ">");
    }
    return boost::lexical_cast< double >(child->GetText());
  }

  // <actuator> element in xml

  static void parseActuator(const ti::TransmissionInfo &src_info,
                            SliderCrankTransmissionInfo *dst_info) {
    // make sure exactly 1 actuator is given
    if (src_info.actuators_.size() != 1) {
      throw ti::TransmissionInterfaceException(
          "Invalid number of actuators: " +
          boost::lexical_cast< std::string >(src_info.actuators_.size()));
    }

    // parse params from the <actuator> element
    const TiXmlElement actuator_el(loadXmlElement(src_info.actuators_[0].xml_element_));
    dst_info->actuator_reduction = parseOptionalValue(actuator_el, "mechanicalReduction", 1.);
  }

  // <joint> element in xml

  static std::size_t findJointByRole(const std::vector< ti::JointInfo > &joints,
                                     const std::string &role) {
    for (std::size_t i = 0; i < joints.size(); ++i) {
      if (joints[i].role_ == role) {
        return i;
      }
    }
    throw ti::TransmissionInterfaceException("No joint with the role of " + role);
  }

  static void parseJoints(const ti::TransmissionInfo &src_info,
                          SliderCrankTransmissionInfo *dst_info) {
    // makes sure exactly 3 joint are given
    if (src_info.joints_.size() != 3) {
      throw ti::TransmissionInterfaceException(
          "Invalid number of joints: " +
          boost::lexical_cast< std::string >(src_info.joints_.size()));
    }

    // parse params from the <joint> element with "crank" role
    dst_info->crank_joint_id = findJointByRole(src_info.joints_, "crank");
    const TiXmlElement crank_joint_el(
        loadXmlElement(src_info.joints_[dst_info->crank_joint_id].xml_element_));
    dst_info->crank_offset = parseOptionalValue(crank_joint_el, "offset", 0.);
    dst_info->crank_length = parseRequiredValue(crank_joint_el, "length");

    // parse params from the <joint> element with "bar" role
    dst_info->bar_joint_id = findJointByRole(src_info.joints_, "bar");
    const TiXmlElement bar_joint_el(
        loadXmlElement(src_info.joints_[dst_info->bar_joint_id].xml_element_));
    dst_info->bar_length = parseRequiredValue(bar_joint_el, "length");

    // parse params from the <joint> element with "slider" role
    dst_info->slider_joint_id = findJointByRole(src_info.joints_, "slider");
    const TiXmlElement slider_joint_el(
        loadXmlElement(src_info.joints_[dst_info->slider_joint_id].xml_element_));
    dst_info->slider_offset_x = parseOptionalValue(slider_joint_el, "offsetX", 0.);
    dst_info->slider_offset_y = parseOptionalValue(slider_joint_el, "offsetY", 0.);
  }
};

} // namespace transmission_interface_extensions

#endif

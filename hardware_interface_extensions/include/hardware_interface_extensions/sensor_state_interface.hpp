#ifndef HARDWARE_INTERFACE_EXTENSIONS_SENSOR_STATE_INTERFACE_HPP
#define HARDWARE_INTERFACE_EXTENSIONS_SENSOR_STATE_INTERFACE_HPP

#include <string>

#include <hardware_interface/internal/hardware_resource_manager.h>
#include <ros/common.h> // for ROS_ASSERT()
#include <sensor_msgs/BatteryState.h>

namespace hardware_interface_extensions {

//
// sensor state handles
//

template < typename Data > class SensorStateHandle {
public:
  SensorStateHandle() : name_(), data_(NULL) {}

  SensorStateHandle(const std::string &name, const Data *const data) : name_(name), data_(data) {}

  virtual ~SensorStateHandle() {}

  std::string getName() const { return name_; }

  ros::Time getStamp() const {
    ROS_ASSERT(data_);
    return data_->header.stamp;
  }

  const Data &getData() const {
    ROS_ASSERT(data_);
    return *data_;
  }

  const Data *getDataPtr() const { return data_; }

private:
  std::string name_;
  const Data * data_;
};

typedef SensorStateHandle< sensor_msgs::BatteryState > BatteryStateHandle;

//
// sensor state interfaces
//

class BatteryStateInterface
    : public hardware_interface::HardwareResourceManager< BatteryStateHandle > {};
} // namespace hardware_interface_extensions

#endif
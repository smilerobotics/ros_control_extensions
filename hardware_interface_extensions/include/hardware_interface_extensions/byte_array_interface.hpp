#ifndef HARDWARE_INTERFACE_EXTENSIONS_BYTE_ARRAY_INTERFACE_HPP
#define HARDWARE_INTERFACE_EXTENSIONS_BYTE_ARRAY_INTERFACE_HPP

#include <cassert>
#include <cstdint> // for std::uint8_t
#include <cstring> // for std::memcpy()
#include <string>
#include <vector>

#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <std_msgs/ByteMultiArray.h>

#include <boost/core/enable_if.hpp>
#include <boost/type_traits/has_trivial_copy.hpp>

namespace hardware_interface_extensions {

// The data type

class ByteArray : public std::vector< std::uint8_t > {
public:
  ByteArray() {}

  ByteArray(const void *const data, const std::size_t length)
      : std::vector< std::uint8_t >(reinterpret_cast< const std::uint8_t * >(data),
                                    reinterpret_cast< const std::uint8_t * >(data) + length) {}

  virtual ~ByteArray() {}

  // Conversions to trivially-copyable (i.e., copyable with memcpy()) types and ROS msg

  template < typename T >
  typename boost::enable_if< boost::has_trivial_copy< T >, T >::type to() const {
    assert(size() == sizeof(T));
    return *reinterpret_cast< const T * >(&front());
  }

  std_msgs::ByteMultiArrayPtr toMsg() const {
    std_msgs::ByteMultiArrayPtr msg(new std_msgs::ByteMultiArray());
    msg->data.resize(size());
    std::memcpy(&msg->data[0], &front(), size());
    return msg;
  }

  // Conversions from trivially-copyable types and ROS msg

  template < typename T >
  static typename boost::enable_if< boost::has_trivial_copy< T >, ByteArray >::type
  from(const T &data) {
    return ByteArray(&data, sizeof(T));
  }

  static ByteArray fromMsg(const std_msgs::ByteMultiArray &data) {
    // TODO: assert input is one dimensional
    return ByteArray(&data.data.front(), data.data.size());
  }
};

// Data handle (name & a pointer to data)

class ByteArrayHandle {
public:
  ByteArrayHandle() : name_(), data_(NULL) {}

  ByteArrayHandle(const std::string &name, ByteArray *const data) : name_(name), data_(data) {
    if (!data) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Byte data pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  void setData(const ByteArray &data) {
    assert(data_);
    *data_ = data;
  }

  const ByteArray &getData() const {
    assert(data_);
    return *data_;
  }

private:
  std::string name_;
  ByteArray *data_;
};

// Data handle managers

class ByteArrayStateInterface
    : public hardware_interface::HardwareResourceManager< ByteArrayHandle,
                                                          hardware_interface::DontClaimResources > {
};

class ByteArrayCommandInterface
    : public hardware_interface::HardwareResourceManager< ByteArrayHandle,
                                                          hardware_interface::ClaimResources > {};
} // namespace hardware_interface_extensions

#endif

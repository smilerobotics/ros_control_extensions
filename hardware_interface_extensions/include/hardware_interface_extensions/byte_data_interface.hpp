#ifndef HARDWARE_INTERFACE_EXTENSIONS_BYTE_DATA_INTERFACE_HPP
#define HARDWARE_INTERFACE_EXTENSIONS_BYTE_DATA_INTERFACE_HPP

#include <cassert>
#include <cstdint> // for std::uint8_t
#include <cstring> // for std::memcpy()
#include <string>
#include <vector>

#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/internal/hardware_resource_manager.h>

namespace hardware_interface_extensions {

class ByteDataHandle {
public:
  ByteDataHandle() : name_(), data_(NULL) {}

  ByteDataHandle(const std::string &name, std::vector< std::uint8_t > *const data)
      : name_(name), data_(data) {
    if (!data) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Byte data pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  void setData(const std::vector< std::uint8_t > &data) {
    assert(data_);
    *data_ = data;
  }

  void setData(const void *const data, const std::size_t length) {
    assert(data_);
    data_->resize(length);
    std::memcpy(&(*data_)[0], data, length);
  }

  std::vector< std::uint8_t > getData() const {
    assert(data_);
    return *data_;
  }

  template < typename T > T getDataAs() const {
    assert(data_);
    return *reinterpret_cast< T * >(&(*data_)[0]);
  }

protected:
  std::string name_;
  std::vector< std::uint8_t > *data_;
};

class ByteDataStateInterface
    : public hardware_interface::HardwareResourceManager< ByteDataHandle,
                                                          hardware_interface::DontClaimResources > {
};

class ByteDataCommandInterface
    : public hardware_interface::HardwareResourceManager< ByteDataHandle,
                                                          hardware_interface::ClaimResources > {};
} // namespace hardware_interface_extensions

#endif

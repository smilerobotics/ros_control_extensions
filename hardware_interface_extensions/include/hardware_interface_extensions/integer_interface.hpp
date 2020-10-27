#ifndef HARDWARE_INTERFACE_EXTENSIONS_INTEGER_INTERFACE_HPP
#define HARDWARE_INTERFACE_EXTENSIONS_INTEGER_INTERFACE_HPP

#include <cassert>
#include <cstdint> // for std::intN_t
#include <string>

#include <hardware_interface/hardware_interface.h> // for HardwareInterfaceException
#include <hardware_interface/internal/hardware_resource_manager.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt8.h>

namespace hardware_interface_extensions {

/////////////////////////////////////////////
// Template of handle & interface for states

template < typename DataT, typename MsgT > class IntegerStateHandle {
public:
  typedef DataT Data;
  typedef MsgT Msg;

public:
  IntegerStateHandle() : name_(), state_(NULL) {}

  IntegerStateHandle(const std::string &name, const Data *const state)
      : name_(name), state_(state) {
    if (!state) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. State data pointer is null.");
    }
  }

  std::string getName() const { return name_; }

  const Data getState() const {
    assert(state_);
    return *state_;
  }

  Msg toMsg() const {
    assert(state_);
    Msg msg;
    msg.data = *state_;
    return msg;
  }

protected:
  std::string name_;
  const Data *state_;
};

template < typename DataT, typename MsgT >
class IntegerStateInterface
    : public hardware_interface::HardwareResourceManager< IntegerStateHandle< DataT, MsgT >,
                                                          hardware_interface::DontClaimResources > {
public:
  typedef IntegerStateHandle< DataT, MsgT > Handle;
};

////////////////////////////
// State handle & interface

// signed 8-64
typedef IntegerStateHandle< std::int8_t, std_msgs::Int8 > Int8StateHandle;
typedef IntegerStateInterface< std::int8_t, std_msgs::Int8 > Int8StateInterface;
typedef IntegerStateHandle< std::int16_t, std_msgs::Int16 > Int16StateHandle;
typedef IntegerStateInterface< std::int16_t, std_msgs::Int16 > Int16StateInterface;
typedef IntegerStateHandle< std::int32_t, std_msgs::Int32 > Int32StateHandle;
typedef IntegerStateInterface< std::int32_t, std_msgs::Int32 > Int32StateInterface;
typedef IntegerStateHandle< std::int64_t, std_msgs::Int64 > Int64StateHandle;
typedef IntegerStateInterface< std::int64_t, std_msgs::Int64 > Int64StateInterface;
// unsigned 8-64
typedef IntegerStateHandle< std::uint8_t, std_msgs::UInt8 > UInt8StateHandle;
typedef IntegerStateInterface< std::uint8_t, std_msgs::UInt8 > UInt8StateInterface;
typedef IntegerStateHandle< std::uint16_t, std_msgs::UInt16 > UInt16StateHandle;
typedef IntegerStateInterface< std::uint16_t, std_msgs::UInt16 > UInt16StateInterface;
typedef IntegerStateHandle< std::uint32_t, std_msgs::UInt32 > UInt32StateHandle;
typedef IntegerStateInterface< std::uint32_t, std_msgs::UInt32 > UInt32StateInterface;
typedef IntegerStateHandle< std::uint64_t, std_msgs::UInt64 > UInt64StateHandle;
typedef IntegerStateInterface< std::uint64_t, std_msgs::UInt64 > UInt64StateInterface;

///////////////////////////////////////////////
// Template of handle & interface for commands

template < typename DataT, typename MsgT >
class IntegerHandle : public IntegerStateHandle< DataT, MsgT > {
private:
  typedef IntegerStateHandle< DataT, MsgT > Base;

public:
  typedef typename Base::Data Data;

public:
  IntegerHandle() : Base(), cmd_(NULL) {}

  IntegerHandle(const Base &base, Data *const cmd) : Base(base), cmd_(cmd) {
    if (!cmd) {
      throw hardware_interface::HardwareInterfaceException(
          "Cannot create handle '" + base.getName() + "'. Command data pointer is null.");
    }
  }

  IntegerHandle(const std::string &name, const Data *const state, Data *const cmd)
      : Base(name, state), cmd_(cmd) {
    if (!cmd) {
      throw hardware_interface::HardwareInterfaceException("Cannot create handle '" + name +
                                                           "'. Command data pointer is null.");
    }
  }

  void setCommand(const Data cmd) {
    assert(cmd_);
    *cmd_ = cmd;
  }

  const Data getCommand() const {
    assert(cmd_);
    return *cmd_;
  }

protected:
  Data *cmd_;
};

template < typename DataT, typename MsgT >
class IntegerInterface
    : public hardware_interface::HardwareResourceManager< IntegerHandle< DataT, MsgT >,
                                                          hardware_interface::ClaimResources > {
public:
  typedef IntegerHandle< DataT, MsgT > Handle;
};

//////////////////////////////
// Command handle & interface

// signed 8-64
typedef IntegerHandle< std::int8_t, std_msgs::Int8 > Int8Handle;
typedef IntegerInterface< std::int8_t, std_msgs::Int8 > Int8Interface;
typedef IntegerHandle< std::int16_t, std_msgs::Int16 > Int16Handle;
typedef IntegerInterface< std::int16_t, std_msgs::Int16 > Int16Interface;
typedef IntegerHandle< std::int32_t, std_msgs::Int32 > Int32Handle;
typedef IntegerInterface< std::int32_t, std_msgs::Int32 > Int32Interface;
typedef IntegerHandle< std::int64_t, std_msgs::Int64 > Int64Handle;
typedef IntegerInterface< std::int64_t, std_msgs::Int64 > Int64Interface;
// unsigned 8-64
typedef IntegerHandle< std::uint8_t, std_msgs::UInt8 > UInt8Handle;
typedef IntegerInterface< std::uint8_t, std_msgs::UInt8 > UInt8Interface;
typedef IntegerHandle< std::uint16_t, std_msgs::UInt16 > UInt16Handle;
typedef IntegerInterface< std::uint16_t, std_msgs::UInt16 > UInt16Interface;
typedef IntegerHandle< std::uint32_t, std_msgs::UInt32 > UInt32Handle;
typedef IntegerInterface< std::uint32_t, std_msgs::UInt32 > UInt32Interface;
typedef IntegerHandle< std::uint64_t, std_msgs::UInt64 > UInt64Handle;
typedef IntegerInterface< std::uint64_t, std_msgs::UInt64 > UInt64Interface;

} // namespace hardware_interface_extensions

#endif

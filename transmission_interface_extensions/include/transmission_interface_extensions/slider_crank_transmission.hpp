#ifndef TRANSMISSION_INTERFACE_EXTENSIONS_SLIDER_CRANK_TRANSMISSION_HPP
#define TRANSMISSION_INTERFACE_EXTENSIONS_SLIDER_CRANK_TRANSMISSION_HPP

#include <cmath>
#include <utility> // for std::size_t

#include <transmission_interface/transmission.h>
#include <transmission_interface_extensions/common_namespaces.hpp>

namespace transmission_interface_extensions {

struct SliderCrankTransmissionInfo {
  // actuator params
  double actuator_reduction;

  // crank joint/link params
  std::size_t crank_joint_id;
  double crank_offset;
  double crank_length;

  // bar joint/link params
  std::size_t bar_joint_id;
  double bar_length;

  // slider joint/link params
  std::size_t slider_joint_id;
  double slider_offset_x;
  double slider_offset_y;
};

class SliderCrankTransmission : public ti::Transmission {
public:
  SliderCrankTransmission(const SliderCrankTransmissionInfo &info)
      : info_(info), present_actuator_pos_(0.), present_slider_pos_(0.) {}

  virtual ~SliderCrankTransmission() {}

  //
  // required interface as a transmission
  //

  virtual void actuatorToJointEffort(const ti::ActuatorData &act_data, ti::JointData &jnt_data) {
    calcJointEffort(*act_data.effort[0], jnt_data.effort[info_.crank_joint_id],
                    jnt_data.effort[info_.bar_joint_id], jnt_data.effort[info_.slider_joint_id]);
  }

  virtual void actuatorToJointVelocity(const ti::ActuatorData &act_data, ti::JointData &jnt_data) {
    calcJointVelocity(*act_data.velocity[0], jnt_data.velocity[info_.crank_joint_id],
                      jnt_data.velocity[info_.bar_joint_id],
                      jnt_data.velocity[info_.slider_joint_id]);
  }

  virtual void actuatorToJointPosition(const ti::ActuatorData &act_data, ti::JointData &jnt_data) {
    calcJointPosition(*act_data.position[0], jnt_data.position[info_.crank_joint_id],
                      jnt_data.position[info_.bar_joint_id],
                      jnt_data.position[info_.slider_joint_id]);

    // assuming this function is called only if converting present actuator to joint positions.
    // I guess this is only method to get the present actuator/joint positions.
    present_actuator_pos_ = *act_data.position[0];
    present_slider_pos_ = *jnt_data.position[info_.slider_joint_id];
  }

  virtual void jointToActuatorEffort(const ti::JointData &jnt_data, ti::ActuatorData &act_data) {
    calcActuatorEffort(*jnt_data.effort[info_.slider_joint_id], act_data.effort[0]);
  }

  virtual void jointToActuatorVelocity(const ti::JointData &jnt_data, ti::ActuatorData &act_data) {
    calcActuatorVelocity(*jnt_data.velocity[info_.slider_joint_id], act_data.velocity[0]);
  }

  virtual void jointToActuatorPosition(const ti::JointData &jnt_data, ti::ActuatorData &act_data) {
    calcActuatorPosition(*jnt_data.position[info_.slider_joint_id], act_data.position[0]);
  }

  virtual std::size_t numActuators() const { return 1; }

  virtual std::size_t numJoints() const { return 3; }

private:
  //
  // actuator -> joint
  //

  // normalize to (-pi, pi]
  static double normalizeAngle(double th) {
    while (th > M_PI) {
      th -= 2. * M_PI;
    }
    while (th <= -M_PI) {
      th += 2. * M_PI;
    }
    return th;
  }

  void calcJointPosition(const double act_pos, double *crank_jnt_pos, double *bar_jnt_pos,
                         double *slider_jnt_pos) const {
    // forward kinematics of slider-crank mechanism in the crank joint frame.
    const double crank_th(normalizeAngle(act_pos / info_.actuator_reduction + info_.crank_offset));
    const double bar_x(info_.crank_length * std::cos(crank_th));
    const double bar_y(info_.crank_length * std::sin(crank_th));
    const double bar2slider_y(info_.slider_offset_y - bar_y);
    const double bar2slider_x(
        std::sqrt(info_.bar_length * info_.bar_length - bar2slider_y * bar2slider_y));
    const double bar_th(std::atan2(bar2slider_y, bar2slider_x));

    // transform from crank to each joint frames
    *crank_jnt_pos = crank_th; // already normalized
    *bar_jnt_pos = normalizeAngle(bar_th - crank_th);
    *slider_jnt_pos = bar_x + bar2slider_x - info_.slider_offset_x;
  }

  void calcJointVelocity(const double act_vel, double *crank_jnt_vel, double *bar_jnt_vel,
                         double *slider_jnt_vel) const {
    double crank_pos, bar_pos, slider_pos;
    calcJointPosition(present_actuator_pos_, &crank_pos, &bar_pos, &slider_pos);
    const double d_act_pos(info_.actuator_reduction * 0.01);
    double crank_pos2, bar_pos2, slider_pos2;
    calcJointPosition(present_actuator_pos_ + d_act_pos, &crank_pos, &bar_pos, &slider_pos2);

    // jnt_vel = d(jnt_pos) / dt
    //         = d(jnt_pos) / d(act_pos) * d(act_pos) / dt
    //         = d(jnt_pos) / d(act_pos) * act_vel
    *crank_jnt_vel = normalizeAngle(crank_pos2 - crank_pos) / d_act_pos * act_vel;
    *bar_jnt_vel = normalizeAngle(bar_pos2 - bar_pos) / d_act_pos * act_vel;
    *slider_jnt_vel = (slider_pos2 - slider_pos) / d_act_pos * act_vel;
  }

  void calcJointEffort(const double act_eff, double *crank_jnt_eff, double *bar_jnt_eff,
                       double *slider_jnt_eff) const {
    double crank_pos, bar_pos, slider_pos;
    calcJointPosition(present_actuator_pos_, &crank_pos, &bar_pos, &slider_pos);
    const double d_act_pos(info_.actuator_reduction * 0.01);
    double crank_pos2, bar_pos2, slider_pos2;
    calcJointPosition(present_actuator_pos_ + d_act_pos, &crank_pos2, &bar_pos2, &slider_pos2);

    // From virtual work priciple,
    //   slider_eff * d(slider_pos) = act_eff * d(act_pos)
    //   slider_eff = d(act_pos) / d(slider_pos) * act_eff
    // where assuming crank & bar do no work
    *crank_jnt_eff = 0.;
    *bar_jnt_eff = 0.;
    *slider_jnt_eff = d_act_pos / (slider_pos2 - slider_pos) * act_eff;
  }

  //
  // slider joint -> actuator
  //

  void calcActuatorPosition(const double slider_jnt_pos, double *act_pos) const {
    // inverse kinematics of slider-crank mechanism in the crank joint frame
    const double slider_x(slider_jnt_pos + info_.slider_offset_x);
    const double slider_r(
        std::sqrt(slider_x * slider_x + info_.slider_offset_y * info_.slider_offset_y));
    const double crank_th(
        normalizeAngle(std::atan2(info_.slider_offset_y, slider_x) +
                       std::acos((slider_r * slider_r + info_.crank_length * info_.crank_length -
                                  info_.bar_length * info_.bar_length) /
                                 (2. * slider_r * info_.crank_length))));

    // simple transmission from crank joint to actuator
    *act_pos = (crank_th - info_.crank_offset) * info_.actuator_reduction;
  }

  void calcActuatorVelocity(const double slider_jnt_vel, double *act_vel) const {
    double act_pos;
    calcActuatorPosition(present_slider_pos_, &act_pos);
    const double d_slider_pos(0.01);
    double act_pos2;
    calcActuatorPosition(present_slider_pos_ + d_slider_pos, &act_pos2);

    // act_vel = d(act_pos) / dt
    //         = d(act_pos) / d(slider_pos) * d(slider_pos) / dt
    //         = d(act_pos) / d(slider_pos) * slider_vel
    *act_vel = (act_pos2 - act_pos) / d_slider_pos * slider_jnt_vel;
  }

  void calcActuatorEffort(const double slider_jnt_eff, double *act_eff) const {
    double act_pos;
    calcActuatorPosition(present_slider_pos_, &act_pos);
    const double d_slider_pos(0.01);
    double act_pos2;
    calcActuatorPosition(present_slider_pos_ + d_slider_pos, &act_pos2);

    // From virtual work priciple,
    //   act_eff * d(act_pos) = slider_eff * d(slider_pos)
    //   act_eff = d(slider_pos) / d(act_pos) * slider_eff
    *act_eff = d_slider_pos / (act_pos2 - act_pos) * slider_jnt_eff;
  }

private:
  const SliderCrankTransmissionInfo info_;
  double present_actuator_pos_, present_slider_pos_;
};

} // namespace transmission_interface_extensions

#endif

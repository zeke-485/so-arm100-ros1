#ifndef SO_ARM_100_INTERFACE_H
#define SO_ARM_100_INTERFACE_H

#include <ros/ros.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>

#include "SCServo_Linux/SCServo.h"
#include <vector>
#include <string>
#include <mutex>

namespace so_arm_100_hardware
{

class SOARM100Interface : public hardware_interface::RobotHW
{
public:
  SOARM100Interface();
  ~SOARM100Interface();

  bool init(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void read(const ros::Time& time, const ros::Duration& period);
  void write(const ros::Time& time, const ros::Duration& period);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher joint_state_pub_;
  ros::ServiceServer torque_srv_;

  std::vector<std::string> joint_names_;
  std::vector<double> pos_, vel_, eff_, cmd_;

  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::PositionJointInterface jnt_pos_interface_;

  std::mutex feedback_mutex_;
  sensor_msgs::JointState last_feedback_;
  
#include "SCServo_Linux/SCServo.h"
#include "SCServo_Linux/SCSCL.h"
  SCSCL scs_;

  bool use_serial_;
  std::string serial_port_;
  int serial_baudrate_;
  std::string calibration_file_;
  bool torque_enabled_;

  bool toggleTorque(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  double ticksToRad(int ticks, size_t idx);
  int radToTicks(double rad, size_t idx);
};

} // namespace

#endif

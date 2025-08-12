#include "so_arm_100_hardware/so_arm_100_interface.h"
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <unistd.h>

namespace so_arm_100_hardware
{

SOARM100Interface::SOARM100Interface()
  : use_serial_(true), serial_baudrate_(1000000), torque_enabled_(true) {}

SOARM100Interface::~SOARM100Interface() {}

bool SOARM100Interface::init(ros::NodeHandle& nh, ros::NodeHandle& pnh)
{
  nh_ = nh; pnh_ = pnh;
  pnh_.param("use_serial", use_serial_, true);
  pnh_.param("serial_port", serial_port_, std::string("/dev/ttyACM1"));
  pnh_.param("serial_baudrate", serial_baudrate_, 1000000);
  pnh_.param("calibration_file", calibration_file_, std::string(""));

  joint_names_ = {"shoulder_pan","shoulder_lift","elbow_flex","wrist_flex","wrist_roll","gripper"};
  size_t n = joint_names_.size();
  pos_.assign(n,0.0); vel_.assign(n,0.0); eff_.assign(n,0.0); cmd_.assign(n,0.0);

  for(size_t i=0;i<n;i++){
    hardware_interface::JointStateHandle state_handle(joint_names_[i], &pos_[i], &vel_[i], &eff_[i]);
    jnt_state_interface_.registerHandle(state_handle);
    hardware_interface::JointHandle pos_handle(jnt_state_interface_.getHandle(joint_names_[i]), &cmd_[i]);
    jnt_pos_interface_.registerHandle(pos_handle);
  }
  registerInterface(&jnt_state_interface_);
  registerInterface(&jnt_pos_interface_);

  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states",10);
  torque_srv_ = nh_.advertiseService("toggle_torque", &SOARM100Interface::toggleTorque, this);

  if (use_serial_) {
    if (!scs_.begin(serial_baudrate_, serial_port_.c_str())) {
      ROS_WARN("SCServo begin failed, running without real hardware");
    } else {
      for(size_t i=0;i<n;i++){
        int id = i+1;
        scs_.FeedBack(id);
        int p = scs_.ReadPos(id);
        if(p!=-1) pos_[i] = ticksToRad(p,i);
        cmd_[i] = pos_[i];
        usleep(2000);
      }
    }
  }
  ROS_INFO("SOARM100Interface init done");
  return true;
}

void SOARM100Interface::read(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  if (use_serial_) {
    for (size_t i=0;i<joint_names_.size();++i){
      int id = i+1;
      scs_.FeedBack(id);                 // fills internal mem buffer
      int p = scs_.ReadPos(id);          // read position
      if (p!=-1) pos_[i] = ticksToRad(p,i);
      usleep(1000);
    }
  }
  sensor_msgs::JointState js;
  js.header.stamp = ros::Time::now();
  js.name = joint_names_;
  js.position = pos_;
  js.velocity = vel_;
  js.effort = eff_;
  joint_state_pub_.publish(js);
}

void SOARM100Interface::write(const ros::Time& /*time*/, const ros::Duration& /*period*/)
{
  if (use_serial_ && torque_enabled_) {
    for(size_t i=0;i<cmd_.size();++i){
      int id = i+1;
      int ticks = radToTicks(cmd_[i], i);

      // Use SCSCL RegWritePos (queue the target position)
      // signature: RegWritePos(u8 ID, u16 Position, u16 Time, u16 Speed = 0)
      scs_.RegWritePos(static_cast<uint8_t>(id), static_cast<uint16_t>(ticks), 4500, 0);
      usleep(1000);
    }
    // Execute all queued RegWritePos commands
    // RegWriteAction is in the lower-level serial API (should be available)
    scs_.RegWriteAction();
  }
}

bool SOARM100Interface::toggleTorque(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
  torque_enabled_ = req.data;
  if (use_serial_) {
    for(size_t i=0;i<joint_names_.size();++i){
      int id = i+1;
      // SCSCL provides EnableTorque(ID, enable)
      scs_.EnableTorque(static_cast<uint8_t>(id), torque_enabled_ ? 1 : 0);
      usleep(1000);
    }
  }
  res.success = true;
  res.message = torque_enabled_ ? "Torque enabled" : "Torque disabled";
  ROS_INFO("%s", res.message.c_str());
  return true;
}

double SOARM100Interface::ticksToRad(int ticks, size_t /*idx*/)
{
  // SCSCL uses 0..4095 with center ~2048
  return (ticks - 2048) * 2.0 * M_PI / 4096.0;
}

int SOARM100Interface::radToTicks(double rad, size_t /*idx*/)
{
  int t = 2048 + static_cast<int>(rad * 4096.0 / (2.0 * M_PI));
  if (t < 0) t = 0;
  if (t > 4095) t = 4095;
  return t;
}

} // namespace so_arm_100_hardware

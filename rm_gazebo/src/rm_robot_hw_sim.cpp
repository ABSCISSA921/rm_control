/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, Qiayuan Liao
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

//
// Created by qiayuan on 2/10/21.
//

#include "rm_gazebo/rm_robot_hw_sim.h"

#include <gazebo_ros_control/gazebo_ros_control_plugin.h>

#include <set>

namespace rm_gazebo
{
bool RmRobotHWSim::initSim(const std::string& robot_namespace, ros::NodeHandle model_nh,
                           gazebo::physics::ModelPtr parent_model, const urdf::Model* urdf_model,
                           std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  bool ret = DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions);
  if (!ret)
    return false;

  hardware_interface::JointStateInterface* default_joint_state_interface =
      get<hardware_interface::JointStateInterface>();
  if (default_joint_state_interface == nullptr)
  {
    ROS_ERROR("DefaultRobotHWSim did not register a JointStateInterface");
    return false;
  }
  for (const auto& joint_name : default_joint_state_interface->getNames())
    combined_joint_state_interface_.registerHandle(default_joint_state_interface->getHandle(joint_name));
  registerPassiveJoints(parent_model, urdf_model, transmissions);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&combined_joint_state_interface_);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&robot_state_interface_);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&imu_sensor_interface_);
  gazebo_ros_control::DefaultRobotHWSim::registerInterface(&rm_imu_sensor_interface_);
  XmlRpc::XmlRpcValue xml_rpc_value;

  if (!model_nh.getParam("imus", xml_rpc_value))
    ROS_WARN("No imu specified");
  else
    parseImu(xml_rpc_value, parent_model);
  world_ = parent_model->GetWorld();  // For gravity
  switch_imu_service_ = model_nh.advertiseService("switch_imu_status", switchImuStatus);
  setupVelocityTruth(model_nh, parent_model);
  return ret;
}

void RmRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  gazebo_ros_control::DefaultRobotHWSim::readSim(time, period);
  for (auto& joint : passive_joint_datas_)
  {
    joint.position = joint.joint_ptr->Position(0);
    joint.velocity = joint.joint_ptr->GetVelocity(0);
    joint.effort = joint.joint_ptr->GetForce(0);
  }

  if (!disable_imu_)
  {
    for (auto& imu : imu_datas_)
    {
      // TODO(qiayuan) Add noise
      ignition::math::Pose3d pose = imu.link_ptr->WorldPose();
      imu.time_stamp = time;
      imu.ori[0] = pose.Rot().X();
      imu.ori[1] = pose.Rot().Y();
      imu.ori[2] = pose.Rot().Z();
      imu.ori[3] = pose.Rot().W();
      ignition::math::Vector3d rate = imu.link_ptr->RelativeAngularVel();
      imu.angular_vel[0] = rate.X();
      imu.angular_vel[1] = rate.Y();
      imu.angular_vel[2] = rate.Z();

      ignition::math::Vector3d gravity = { 0., 0., -9.81 };
      ignition::math::Vector3d accel = imu.link_ptr->RelativeLinearAccel() - pose.Rot().RotateVectorReverse(gravity);
      imu.linear_acc[0] = accel.X();
      imu.linear_acc[1] = accel.Y();
      imu.linear_acc[2] = accel.Z();
    }
  }

  // Set cmd to zero to avoid crazy soft limit oscillation when not controller loaded
  for (auto& cmd : joint_effort_command_)
    cmd = 0;
  for (auto& cmd : joint_velocity_command_)
    cmd = 0;

  publishVelocityTruth(time, period);
}
void RmRobotHWSim::setupVelocityTruth(ros::NodeHandle& model_nh,
                                      const gazebo::physics::ModelPtr& parent_model)
{
  bool enabled = false;
  model_nh.param("gazebo_velocity_truth_enabled", enabled, false);
  if (!enabled)
    return;

  const std::array<std::string, 4> names = { "left_wheel", "right_wheel", "base_link", "base_imu" };
  auto& message = velocity_truth_message_;
  message.name = { "time", "dt" };
  for (size_t i = 0; i < names.size(); ++i)
  {
    velocity_truth_links_[i] = parent_model->GetLink(names[i]);
    if (!velocity_truth_links_[i])
    {
      ROS_WARN("Velocity truth recording unavailable: Gazebo link %s is absent", names[i].c_str());
      return;  // Recording availability never changes controller initialization or feedback.
    }
    for (const std::string& field : { "position_x", "position_y", "position_z", "velocity_x", "velocity_y",
                                      "velocity_z", "orientation_x", "orientation_y", "orientation_z",
                                      "orientation_w", "angular_velocity_x", "angular_velocity_y", "angular_velocity_z" })
      message.name.push_back(names[i] + "_" + field);
  }
  for (size_t i = 0; i < 2; ++i)
    for (const std::string& axis : { "x", "y", "z" })
      message.name.push_back(names[i] + "_axis_" + axis);
  message.value.resize(message.name.size());
  message.stamp.resize(message.name.size());
  velocity_truth_pub_ = model_nh.advertise<rm_msgs::DebugData>("gazebo_velocity_truth", 100);
}

void RmRobotHWSim::publishVelocityTruth(const ros::Time& time, const ros::Duration& period)
{
  if (!velocity_truth_pub_)
    return;
  auto& message = velocity_truth_message_;
  message.value[0] = time.toSec();
  message.value[1] = period.toSec();
  for (size_t i = 0; i < velocity_truth_links_.size(); ++i)
  {
    const auto& link = velocity_truth_links_[i];
    const auto pose = link->WorldPose();
    // Gazebo WorldLinearVel() returns link-origin velocity, including the CoG offset.
    // In series_legged the wheel link origins are the wheel centers.
    const auto velocity = link->WorldLinearVel();
    const auto angular_velocity = link->WorldAngularVel();
    const std::array<double, 13> values = {
        pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z(), velocity.X(), velocity.Y(), velocity.Z(),
        pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W(),
        angular_velocity.X(), angular_velocity.Y(), angular_velocity.Z()
    };
    for (size_t j = 0; j < values.size(); ++j)
      message.value[2 + i * values.size() + j] = values[j];
    if (i < 2)
    {
      const auto axis = pose.Rot().RotateVector(ignition::math::Vector3d(0., 1., 0.));
      message.value[54 + i * 3] = axis.X();
      message.value[55 + i * 3] = axis.Y();
      message.value[56 + i * 3] = axis.Z();
    }
  }
  for (auto& stamp : message.stamp)
    stamp = time;
  velocity_truth_pub_.publish(message);
}

void RmRobotHWSim::registerPassiveJoints(
    const gazebo::physics::ModelPtr& parent_model, const urdf::Model* urdf_model,
    const std::vector<transmission_interface::TransmissionInfo>& transmissions)
{
  if (urdf_model == nullptr)
  {
    ROS_WARN("Cannot register passive joints without a URDF model");
    return;
  }

  std::set<std::string> transmitted_joint_names;
  for (const auto& transmission : transmissions)
    for (const auto& joint : transmission.joints_)
      transmitted_joint_names.insert(joint.name_);

  for (const auto& entry : urdf_model->joints_)
  {
    const auto& joint_name = entry.first;
    const auto& urdf_joint = entry.second;
    if (urdf_joint == nullptr || transmitted_joint_names.count(joint_name) != 0)
      continue;
    if (urdf_joint->type != urdf::Joint::REVOLUTE && urdf_joint->type != urdf::Joint::CONTINUOUS &&
        urdf_joint->type != urdf::Joint::PRISMATIC)
      continue;

    gazebo::physics::JointPtr gazebo_joint = parent_model->GetJoint(joint_name);
    if (gazebo_joint == nullptr)
    {
      ROS_WARN("Passive URDF joint %s is not present in the Gazebo model", joint_name.c_str());
      continue;
    }

    passive_joint_datas_.push_back(PassiveJointData{ gazebo_joint });
    PassiveJointData& data = passive_joint_datas_.back();
    data.position = gazebo_joint->Position(0);
    data.velocity = gazebo_joint->GetVelocity(0);
    data.effort = gazebo_joint->GetForce(0);
    combined_joint_state_interface_.registerHandle(
        hardware_interface::JointStateHandle(joint_name, &data.position, &data.velocity, &data.effort));
    ROS_INFO("Registered passive Gazebo joint state: %s", joint_name.c_str());
  }
}

void RmRobotHWSim::parseImu(XmlRpc::XmlRpcValue& imu_datas, const gazebo::physics::ModelPtr& parent_model)
{
  ROS_ASSERT(imu_datas.getType() == XmlRpc::XmlRpcValue::TypeStruct);
  for (auto it = imu_datas.begin(); it != imu_datas.end(); ++it)
  {
    if (!it->second.hasMember("frame_id"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated frame id.");
      continue;
    }
    else if (!it->second.hasMember("orientation_covariance_diagonal"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated orientation covariance diagonal.");
      continue;
    }
    else if (!it->second.hasMember("angular_velocity_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated angular velocity covariance.");
      continue;
    }
    else if (!it->second.hasMember("linear_acceleration_covariance"))
    {
      ROS_ERROR_STREAM("Imu " << it->first << " has no associated linear acceleration covariance.");
      continue;
    }
    std::string frame_id = imu_datas[it->first]["frame_id"];
    gazebo::physics::LinkPtr link_ptr = parent_model->GetLink(frame_id);
    if (link_ptr == nullptr)
    {
      ROS_WARN("Imu %s is not specified in urdf.", it->first.c_str());
      continue;
    }
    XmlRpc::XmlRpcValue ori_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(ori_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(ori_cov.size() == 3);
    for (int i = 0; i < ori_cov.size(); ++i)
      ROS_ASSERT(ori_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    XmlRpc::XmlRpcValue angular_cov = imu_datas[it->first]["orientation_covariance_diagonal"];
    ROS_ASSERT(angular_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(angular_cov.size() == 3);
    for (int i = 0; i < angular_cov.size(); ++i)
      ROS_ASSERT(angular_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    XmlRpc::XmlRpcValue linear_cov = imu_datas[it->first]["linear_acceleration_covariance"];
    ROS_ASSERT(linear_cov.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(linear_cov.size() == 3);
    for (int i = 0; i < linear_cov.size(); ++i)
      ROS_ASSERT(linear_cov[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    imu_datas_.push_back((ImuData{
        .link_ptr = link_ptr,
        .time_stamp = {},
        .ori = { 0., 0., 0., 0. },
        .ori_cov = { static_cast<double>(ori_cov[0]), 0., 0., 0., static_cast<double>(ori_cov[1]), 0., 0., 0.,
                     static_cast<double>(ori_cov[2]) },
        .angular_vel = { 0., 0., 0. },
        .angular_vel_cov = { static_cast<double>(angular_cov[0]), 0., 0., 0., static_cast<double>(angular_cov[1]), 0.,
                             0., 0., static_cast<double>(angular_cov[2]) },
        .linear_acc = { 0., 0., 0. },
        .linear_acc_cov = { static_cast<double>(linear_cov[0]), 0., 0., 0., static_cast<double>(linear_cov[1]), 0., 0.,
                            0., static_cast<double>(linear_cov[2]) } }));
    ImuData& imu_data = imu_datas_.back();
    hardware_interface::ImuSensorHandle imu_sensor_handle(it->first, frame_id, imu_data.ori, imu_data.ori_cov,
                                                          imu_data.angular_vel, imu_data.angular_vel_cov,
                                                          imu_data.linear_acc, imu_data.linear_acc_cov);
    imu_sensor_interface_.registerHandle(hardware_interface::ImuSensorHandle(imu_sensor_handle));
    rm_imu_sensor_interface_.registerHandle(rm_control::RmImuSensorHandle(imu_sensor_handle, &imu_data.time_stamp));
  }
}

bool RmRobotHWSim::disable_imu_ = false;
}  // namespace rm_gazebo

PLUGINLIB_EXPORT_CLASS(rm_gazebo::RmRobotHWSim, gazebo_ros_control::RobotHWSim)
GZ_REGISTER_MODEL_PLUGIN(gazebo_ros_control::GazeboRosControlPlugin)  // Default plugin

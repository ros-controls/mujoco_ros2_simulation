/**
 * Copyright (c) 2025, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 *
 * All rights reserved.
 *
 * This software is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <mujoco/mujoco.h>

// Pull in the Simulate class and PhysicsThread/RenderLoop declarations:
#include "glfw_adapter.h"  // for mj::GlfwAdapter
#include "simulate.h"      // must be on your include path, handled by CMake

#include "mujoco_ros2_simulation/data.hpp"

namespace mujoco_ros2_simulation
{
class MujocoSystemInterface : public hardware_interface::SystemInterface
{
public:
  MujocoSystemInterface();
  ~MujocoSystemInterface() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  // Constructs all actuator/joint data containers for the interface
  void register_joints(const hardware_interface::HardwareInfo& info);

  // Constructs all sensor data containers for the interface
  void register_sensors(const hardware_interface::HardwareInfo& info);

  // Sets the initial pose based on URDF params
  void set_initial_pose();

  // For spinning the physics simulation
  void PhysicsLoop();

  // System information
  hardware_interface::HardwareInfo system_info_;
  std::string model_path_;

  // MuJoCo data pointers
  mjModel* mj_model_{ nullptr };
  mjData* mj_data_{ nullptr };

  // For rendering
  mjvCamera cam_;
  mjvOption opt_;
  mjvPerturb pert_;

  // Primary simulate object
  std::unique_ptr<mujoco::Simulate> sim_;

  // Threads for rendering physics and the UI window
  std::thread physics_thread_;
  std::thread ui_thread_;

  // Mutex used inside simulate.h for protecting model/data, we keep a reference
  // here to protect access to shared data.
  std::recursive_mutex* sim_mutex_{ nullptr };

  // Data containers for the HW interface
  std::vector<hardware_interface::StateInterface> state_interfaces_;
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  // Data containers for the HW interface
  std::vector<JointState> joint_states_;
  std::vector<FTSensorData> ft_sensor_data_;
  std::vector<IMUSensorData> imu_sensor_data_;
};

}  // namespace mujoco_ros2_simulation

/*!*******************************************************************************************
 *  \file       follow_path_plugin_position.cpp
 *  \brief      This file contains the implementation of the follow path behaviour position plugin
 *  \authors    Rafael Pérez Seguí
 *
 *  \copyright  Copyright (c) 2022 Universidad Politécnica de Madrid
 *              All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************************************************************************/

#include "follow_path_base.hpp"
#include "motion_reference_handlers/position_motion.hpp"

namespace follow_path_plugin_position {
class Plugin : public follow_path_base::FollowPathBase {
private:
  std::shared_ptr<as2::motionReferenceHandlers::PositionMotion> position_motion_handler_ = nullptr;

public:
  void ownInit() {
    position_motion_handler_ =
        std::make_shared<as2::motionReferenceHandlers::PositionMotion>(node_ptr_);
  }

  bool on_deactivate(const std::shared_ptr<std::string> &message) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Goal canceled");
    return true;
  }

  bool on_pause(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path paused");
    sendHover();
    return true;
  }

  bool on_resume(const std::shared_ptr<std::string> &message) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path resumed");
    return true;
  }

  bool own_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path goal accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f", goal->max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with yaw mode: %f", goal->yaw.mode);

    path_ids_.reserve(goal->path.size());
    for (auto &point : goal->path) {
      RCLCPP_INFO(node_ptr_->get_logger(), "Follow path to point %d: %f, %f, %f", point.id,
                  point.pose.position.x, point.pose.position.y, point.pose.position.z);
      path_ids_.push_back(point.id);
    }
    path_ids_remaining_ = path_ids_;
    return true;
  }

  bool own_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path modiy accepted");
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with speed: %f", goal->max_speed);
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path with yaw mode: %f", goal->yaw.mode);

    for (auto &point : goal->path) {
      if (std::find(path_ids_.begin(), path_ids_.end(), point.id) == path_ids_.end()) {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path modify point %d: %f, %f, %f", point.id,
                    point.pose.position.x, point.pose.position.y, point.pose.position.z);
      } else {
        RCLCPP_INFO(node_ptr_->get_logger(), "Follow path add point %d: %f, %f, %f", point.id,
                    point.pose.position.x, point.pose.position.y, point.pose.position.z);
        path_ids_.push_back(point.id);
        path_ids_remaining_.push_back(point.id);
      }
    }
    return true;
  }

  as2_behavior::ExecutionStatus own_run() override {
    if (checkGoalCondition()) {
      result_.follow_path_success = true;
      RCLCPP_INFO(node_ptr_->get_logger(), "Goal succeeded");
      return as2_behavior::ExecutionStatus::SUCCESS;
    }

    if (!position_motion_handler_->sendPositionCommandWithYawAngle(
            "earth", goal_.target_pose.point.x, goal_.target_pose.point.y,
            goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed, goal_.max_speed,
            goal_.max_speed)) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "Follow path PLUGIN: Error sending position command");
      result_.follow_path_success = false;
      return as2_behavior::ExecutionStatus::FAILURE;
    }

    return as2_behavior::ExecutionStatus::RUNNING;
  }

  void own_execution_end(const as2_behavior::ExecutionStatus &state) {
    RCLCPP_INFO(node_ptr_->get_logger(), "Follow path end");
    if (state == as2_behavior::ExecutionStatus::SUCCESS) {
      // Leave the drone in the last position

      if (position_motion_handler_->sendPositionCommandWithYawAngle(
              "earth", goal_.target_pose.point.x, goal_.target_pose.point.y,
              goal_.target_pose.point.z, goal_.yaw.angle, "earth", goal_.max_speed, goal_.max_speed,
              goal_.max_speed))
        return;
    }
    sendHover();
    return;
  }

  Eigen::Vector3d getTargetPosition() {
    for (auto &waypoint : goal_.path) {
      if (waypoint.id == path_ids_remaining_.front()) {
        return Eigen::Vector3d(waypoint.pose.position.x, waypoint.pose.position.y,
                               waypoint.pose.position.z);
      }
    }
    return Eigen::Vector3d(0, 0, 0);
  }

  bool sendCommand(as2_msgs::msg::PoseWithID &waypoint) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "earth";
    pose.header.stamp    = node_ptr_->now();
    pose.pose.position.x = waypoint.pose.position.x;
    pose.pose.position.y = waypoint.pose.position.y;
    pose.pose.position.z = waypoint.pose.position.z;

    geometry_msgs::msg::TwistStamped twist;
    twist.header.frame_id = "earth";
    twist.header.stamp    = node_ptr_->now();
    twist.twist.linear.x  = goal_.max_speed;
    twist.twist.linear.y  = goal_.max_speed;
    twist.twist.linear.z  = goal_.max_speed;

    return position_motion_handler_->sendPositionCommandWithYawAngle(pose, twist);
  }

private:
  std::vector<std::string> path_ids_;
  std::vector<std::string> path_ids_remaining_;

  bool checkGoalCondition() {
    if (distance_measured_) {
      if (fabs(feedback_.actual_distance_to_next_waypoint) < params_.follow_path_threshold) {
        path_ids_remaining_.erase(path_ids_remaining_.begin());
        if (path_ids_remaining_.empty()) {
          return true;
        }
      }
    }
    return false;
  }

};  // Plugin class
}  // namespace follow_path_plugin_position

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(follow_path_plugin_position::Plugin, follow_path_base::FollowPathBase)

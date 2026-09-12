// Copyright 2026 G1 Locomanipulation contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <g1_interfaces/action/pick.hpp>
#include <g1_interfaces/srv/grasp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;
using Pick = g1_interfaces::action::Pick;
using Handle = rclcpp_action::ServerGoalHandle<Pick>;

// Only one task owns the arms. A captured or uncertain object latches ownership
// until a future Place/recovery implementation explicitly releases it.
class PickServer
{
public:
  explicit PickServer(const rclcpp::Node::SharedPtr & node)
  : node_(node)
  {
    grasp_ = node_->create_client<g1_interfaces::srv::Grasp>("/g1/grasp");
    server_ = rclcpp_action::create_server<Pick>(
      node_, "/g1/pick",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Pick::Goal> goal) {
        const auto & p = goal->grasp_pose.pose.position;
        const auto & q = goal->grasp_pose.pose.orientation;
        const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (goal->object_id.empty() ||
          (goal->arm != "left_arm" && goal->arm != "right_arm") ||
          goal->grasp_pose.header.frame_id != "pelvis" ||
          !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
          !std::isfinite(norm) || std::abs(norm - 1.0) > 0.01 ||
          !std::isfinite(goal->approach_distance) ||
          !std::isfinite(goal->lift_distance) ||
          goal->approach_distance <= 0.0 || goal->approach_distance > 0.25 ||
          goal->lift_distance <= 0.0 || goal->lift_distance > 0.25 ||
          held_or_uncertain_ || busy_.exchange(true))
        {
          return rclcpp_action::GoalResponse::REJECT;
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](const std::shared_ptr<Handle>) {
        // Cooperative cancellation at stage boundaries; never drops an object.
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<Handle> goal) {
        if (worker_.joinable()) {
          worker_.join();
        }
        worker_ = std::thread([this, goal]() {execute(goal);});
      });
  }

  ~PickServer()
  {
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  void execute(const std::shared_ptr<Handle> & handle)
  {
    auto result = std::make_shared<Pick::Result>();
    auto feedback = [&](const std::string & stage) {
        if (!rclcpp::ok() || handle->is_canceling()) {
          throw std::runtime_error("Canceled at stage boundary");
        }
        auto message = std::make_shared<Pick::Feedback>();
        message->stage = stage;
        handle->publish_feedback(message);
      };
    try {
      feedback("checking_backend");
      if (!grasp_->wait_for_service(2s)) {
        throw std::runtime_error(
                "No physical grasp backend at /g1/grasp. Rigid hands cannot grasp.");
      }
      const auto goal = handle->get_goal();
      moveit::planning_interface::PlanningSceneInterface scene;
      if (scene.getObjects({goal->object_id}).empty()) {
        throw std::runtime_error("Object missing from MoveIt planning scene");
      }
      moveit::planning_interface::MoveGroupInterface group(node_, goal->arm);
      const std::string hand = goal->arm == "left_arm" ?
        "left_rubber_hand" : "right_rubber_hand";
      group.setEndEffectorLink(hand);
      group.setPoseReferenceFrame("pelvis");
      group.setPlanningTime(5.0);
      group.setMaxVelocityScalingFactor(0.1);
      group.setMaxAccelerationScalingFactor(0.1);

      auto move = [&](const geometry_msgs::msg::Pose & pose, const std::string & stage) {
          feedback(stage + ":planning");
          group.setStartStateToCurrentState();
          group.setPoseTarget(pose, hand);
          moveit::planning_interface::MoveGroupInterface::Plan plan;
          if (!static_cast<bool>(group.plan(plan))) {
            throw std::runtime_error(stage + ": planning failed");
          }
          feedback(stage + ":executing");
          if (!static_cast<bool>(group.execute(plan))) {
            throw std::runtime_error(stage + ": execution failed");
          }
        };

      auto pose = goal->grasp_pose.pose;
      pose.position.x -= goal->approach_distance;
      move(pose, "pregrasp");
      move(goal->grasp_pose.pose, "approach");
      feedback("physical_grasp");
      auto request = std::make_shared<g1_interfaces::srv::Grasp::Request>();
      request->object_id = goal->object_id;
      request->hand_link = hand;
      auto response = grasp_->async_send_request(request);
      // Timeout is an unknown physical state, not a failed/open grasp.
      held_or_uncertain_ = true;
      if (response.wait_for(10s) != std::future_status::ready) {
        grasp_->remove_pending_request(response);
        throw std::runtime_error("Grasp timed out: physical state UNKNOWN; recovery required");
      }
      const auto reply = response.get();
      if (!reply->success) {
        throw std::runtime_error("Grasp not confirmed; recovery required: " + reply->message);
      }
      feedback("attaching_planning_object");
      if (!group.attachObject(goal->object_id, hand, std::vector<std::string>{hand})) {
        throw std::runtime_error("Physical grasp succeeded but planning attachment failed");
      }
      bool attached = false;
      for (int attempt = 0; attempt < 20 && rclcpp::ok(); ++attempt) {
        if (!scene.getAttachedObjects({goal->object_id}).empty()) {
          attached = true;
          break;
        }
        std::this_thread::sleep_for(50ms);
      }
      result->object_attached = attached;
      if (!attached) {
        throw std::runtime_error("Planning scene did not confirm attachment; recovery required");
      }
      pose = goal->grasp_pose.pose;
      pose.position.z += goal->lift_distance;
      move(pose, "lift");
      feedback("complete");
      result->success = true;
      result->message = "Grasp confirmed, object attached and lift executed";
      handle->succeed(result);
    } catch (const std::exception & error) {
      result->message = error.what();
      if (rclcpp::ok()) {
        if (handle->is_canceling()) {
          handle->canceled(result);
        } else {
          handle->abort(result);
        }
      }
    }
    busy_ = false;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<g1_interfaces::srv::Grasp>::SharedPtr grasp_;
  rclcpp_action::Server<Pick>::SharedPtr server_;
  std::atomic<bool> busy_{false};
  std::atomic<bool> held_or_uncertain_{false};
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "g1_pick_server", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  PickServer server(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

// Copyright 2026 G1 Locomanipulation contributors
// Licensed under the Apache License, Version 2.0.

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace
{
struct Options
{
  std::string group{"left_arm"};
  bool named_home{false};
  bool plan_only{false};
  bool add_table{true};
  double x{0.35};
  double y{0.25};
  double z{1.05};
  double roll{0.0};
  double pitch{1.5708};
  double yaw{0.0};
};

double number(const char * value, const std::string & flag)
{
  try {
    std::size_t consumed = 0;
    const double result = std::stod(value, &consumed);
    if (consumed != std::string(value).size() || !std::isfinite(result)) {
      throw std::invalid_argument("not finite");
    }
    return result;
  } catch (const std::exception &) {
    throw std::runtime_error(flag + " requiere un número válido");
  }
}

void print_usage(const char * program)
{
  std::cout << "Uso: " << program << " [opciones]\n"
            << "  --group left_arm|right_arm\n"
            << "  --pose X Y Z ROLL PITCH YAW   (metros y radianes, frame world)\n"
            << "  --named-home                  usa la pose SRDF 'home'\n"
            << "  --plan-only                   planifica sin ejecutar\n"
            << "  --no-table                    no agrega la mesa de colisión\n";
}

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      print_usage(argv[0]);
      std::exit(EXIT_SUCCESS);
    } else if (argument == "--group") {
      if (++index >= argc) {throw std::runtime_error("falta el valor de --group");}
      options.group = argv[index];
    } else if (argument == "--pose") {
      if (index + 6 >= argc) {throw std::runtime_error("--pose requiere 6 valores");}
      options.x = number(argv[++index], "--pose");
      options.y = number(argv[++index], "--pose");
      options.z = number(argv[++index], "--pose");
      options.roll = number(argv[++index], "--pose");
      options.pitch = number(argv[++index], "--pose");
      options.yaw = number(argv[++index], "--pose");
    } else if (argument == "--named-home") {
      options.named_home = true;
    } else if (argument == "--plan-only") {
      options.plan_only = true;
    } else if (argument == "--no-table") {
      options.add_table = false;
    } else if (argument.rfind("--ros-args", 0) == 0) {
      break;
    } else {
      throw std::runtime_error("opción desconocida: " + argument);
    }
  }
  if (options.group != "left_arm" && options.group != "right_arm") {
    throw std::runtime_error("--group debe ser left_arm o right_arm");
  }
  return options;
}

void add_table(moveit::planning_interface::PlanningSceneInterface & scene)
{
  moveit_msgs::msg::CollisionObject table;
  table.header.frame_id = "world";
  table.id = "work_table";

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {0.70, 1.20, 0.05};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = 0.55;
  pose.position.z = 0.72;
  table.primitives.push_back(box);
  table.primitive_poses.push_back(pose);
  table.operation = moveit_msgs::msg::CollisionObject::ADD;
  scene.applyCollisionObject(table);
}
}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  try {
    options = parse_options(argc, argv);
  } catch (const std::exception & error) {
    std::cerr << "Error: " << error.what() << "\n";
    print_usage(argv[0]);
    return EXIT_FAILURE;
  }

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "g1_move_to_pose", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  int return_code = EXIT_FAILURE;
  try {
    moveit::planning_interface::MoveGroupInterface move_group(node, options.group);
    moveit::planning_interface::PlanningSceneInterface scene;
    move_group.setPoseReferenceFrame("world");
    move_group.setPlanningTime(8.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.2);
    move_group.setMaxAccelerationScalingFactor(0.2);
    move_group.setPlannerId("RRTConnectkConfigDefault");
    move_group.setStartStateToCurrentState();

    if (options.add_table) {
      add_table(scene);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      RCLCPP_INFO(node->get_logger(), "Mesa agregada a la escena de colisiones");
    }

    if (options.named_home) {
      move_group.setNamedTarget("home");
    } else {
      geometry_msgs::msg::Pose target;
      target.position.x = options.x;
      target.position.y = options.y;
      target.position.z = options.z;
      tf2::Quaternion orientation;
      orientation.setRPY(options.roll, options.pitch, options.yaw);
      target.orientation.x = orientation.x();
      target.orientation.y = orientation.y();
      target.orientation.z = orientation.z();
      target.orientation.w = orientation.w();
      move_group.setPoseTarget(target);
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group.plan(plan));
    if (!planned) {
      RCLCPP_ERROR(node->get_logger(), "MoveIt no encontró una trayectoria libre de colisiones");
    } else if (options.plan_only) {
      RCLCPP_INFO(node->get_logger(), "Plan válido generado; --plan-only evita ejecutarlo");
      return_code = EXIT_SUCCESS;
    } else if (static_cast<bool>(move_group.execute(plan))) {
      RCLCPP_INFO(node->get_logger(), "Trayectoria ejecutada por ros2_control");
      return_code = EXIT_SUCCESS;
    } else {
      RCLCPP_ERROR(node->get_logger(), "El controlador no pudo ejecutar la trayectoria");
    }
    move_group.clearPoseTargets();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(node->get_logger(), "Fallo en MoveIt: %s", error.what());
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();
  return return_code;
}

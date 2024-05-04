#include <moveit2>
#include <iostream>
#include <Eigen>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "std_msgs/msg/string.hpp"


static const std::string PLANNING_GROUP = "panda_arm";
moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const moveit::core::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial", move_group.getRobotModel());
RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
RCLCPP_INFO(LOGGER, "Available Planning Groups:");
std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

std::vector<geometry_msgs::msg::Pose> waypoints;
moveit_msgs::msg::RobotTrajectory trajectory;
const double jump_threshold = 5.0;
const double eef_step = 0.01;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
RCLCPP_INFO(LOGGER, "Visualizing plan 4 (Cartesian path) (%.2f%% acheived)", fraction * 100.0);
visual_tools.deleteAllMarkers();
visual_tools.publishText(text_pose, "Cartesian_Path", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
visual_tools.publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL);
for (std::size_t i = 0; i < waypoints.size(); ++i)
  visual_tools.publishAxisLabeled(waypoints[i], "pt" + std::to_string(i), rviz_visual_tools::SMALL);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
move_group.execute(trajectory);
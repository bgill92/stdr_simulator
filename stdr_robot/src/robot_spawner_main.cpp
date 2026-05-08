/** @file robot_spawner — calls SpawnRobot then runs StdrRobotNode in-process.
 *
 *  This replaces the ROS1 robot_handler. The two-step approach is required in
 *  ROS2: the server must assign a name (robot0, robot1, …) before the robot
 *  node can register itself under that name. */

#include "stdr_robot/stdr_robot_node.hpp"

#include <stdr_parser/parser.hpp>

#include <stdr_msgs/action/spawn_robot.hpp>
#include <stdr_msgs/msg/robot_msg.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tl_expected/expected.hpp>

#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace
{

using SpawnRobot = stdr_msgs::action::SpawnRobot;

constexpr std::chrono::seconds kActionServerWaitTimeout{ 30 };
constexpr std::chrono::seconds kGoalAcceptTimeout{ 10 };
constexpr std::chrono::seconds kGoalResultTimeout{ 30 };

void print_usage(const char* program)
{
  std::cerr << "Usage: " << program << " [OPTIONS]\n"
            << "Options:\n"
            << "  --description <yaml_path>   Path to the robot YAML file (required)\n"
            << "  --x <float>                 Initial x position (default: 0)\n"
            << "  --y <float>                 Initial y position (default: 0)\n"
            << "  --theta <float>             Initial heading in radians (default: 0)\n"
            << "  --specs-dir <path>          Specs directory (default: stdr_resources share)\n"
            << "  --base-dir <path>           Base directory for resolving references"
               " (default: stdr_resources share/resources)\n"
            << "  --help                      Show this help message\n";
}

struct SpawnArgs
{
  std::string description_path;
  double x{ 0.0 };
  double y{ 0.0 };
  double theta{ 0.0 };
  std::string specs_dir;
  std::string base_dir;
};

/** Parse application-level CLI arguments, stopping before --ros-args. */
[[nodiscard]] tl::expected<SpawnArgs, std::string> parse_args(int argc, char* argv[])
{
  const std::string stdr_resources_share = ament_index_cpp::get_package_share_directory("stdr_resources");

  SpawnArgs args;
  args.specs_dir = stdr_resources_share + "/resources/specifications";
  args.base_dir = stdr_resources_share + "/resources";

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];

    if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      // Signal help exit via a sentinel error; caller exits with success.
      return tl::unexpected<std::string>("__help__");
    }
    if (arg.rfind("--ros-args", 0) == 0 || arg == "--")
    {
      // ROS2 injects --ros-args and related tokens; stop parsing here.
      break;
    }

    // Options that take a value.
    auto require_next = [&](const char* flag) -> tl::expected<std::string, std::string> {
      if (i + 1 >= argc)
      {
        return tl::unexpected<std::string>(std::string(flag) + " requires a value argument");
      }
      return std::string(argv[++i]);
    };

    if (arg == "--description")
    {
      tl::expected<std::string, std::string> val = require_next("--description");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      args.description_path = val.value();
    }
    else if (arg == "--x")
    {
      tl::expected<std::string, std::string> val = require_next("--x");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      try
      {
        args.x = std::stod(val.value());
      }
      catch (const std::exception& e)
      {
        return tl::unexpected<std::string>("--x: invalid float: " + val.value());
      }
    }
    else if (arg == "--y")
    {
      tl::expected<std::string, std::string> val = require_next("--y");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      try
      {
        args.y = std::stod(val.value());
      }
      catch (const std::exception& e)
      {
        return tl::unexpected<std::string>("--y: invalid float: " + val.value());
      }
    }
    else if (arg == "--theta")
    {
      tl::expected<std::string, std::string> val = require_next("--theta");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      try
      {
        args.theta = std::stod(val.value());
      }
      catch (const std::exception& e)
      {
        return tl::unexpected<std::string>("--theta: invalid float: " + val.value());
      }
    }
    else if (arg == "--specs-dir")
    {
      tl::expected<std::string, std::string> val = require_next("--specs-dir");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      args.specs_dir = val.value();
    }
    else if (arg == "--base-dir")
    {
      tl::expected<std::string, std::string> val = require_next("--base-dir");
      if (!val)
      {
        return tl::unexpected(val.error());
      }
      args.base_dir = val.value();
    }
    else
    {
      return tl::unexpected<std::string>("Unknown argument: " + arg);
    }
  }

  if (args.description_path.empty())
  {
    return tl::unexpected<std::string>("--description is required");
  }

  return args;
}

/**
 * @brief Call the SpawnRobot action server and return the assigned robot name.
 *
 * Blocks until the action completes or an error occurs. The action server is
 * waited on for up to kActionServerWaitTimeout seconds before giving up.
 */
[[nodiscard]] tl::expected<std::string, std::string> spawn_robot(stdr_msgs::msg::RobotMsg robot_msg)
{
  // A short-lived node dedicated to the action client. Destroyed before the
  // robot node is constructed so there are no naming conflicts.
  std::shared_ptr<rclcpp::Node> client_node = std::make_shared<rclcpp::Node>("robot_spawner_client");

  rclcpp_action::Client<SpawnRobot>::SharedPtr spawn_client =
      rclcpp_action::create_client<SpawnRobot>(client_node, "stdr_server/spawn_robot");

  // Wait for the action server to come up, spinning callbacks in the meantime.
  const rclcpp::Time deadline = client_node->now() + rclcpp::Duration(kActionServerWaitTimeout);

  while (!spawn_client->action_server_is_ready())
  {
    if (client_node->now() >= deadline)
    {
      return tl::unexpected<std::string>("Timed out waiting for stdr_server/spawn_robot action server");
    }
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  SpawnRobot::Goal goal;
  goal.description = std::move(robot_msg);

  // Send the goal.
  std::shared_future<rclcpp_action::ClientGoalHandle<SpawnRobot>::SharedPtr> goal_handle_future =
      spawn_client->async_send_goal(goal);

  // Spin until the goal is accepted, with timeout and shutdown guard.
  const rclcpp::Time goal_accept_deadline = client_node->now() + rclcpp::Duration(kGoalAcceptTimeout);

  while (goal_handle_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    if (!rclcpp::ok())
    {
      return tl::unexpected<std::string>("Interrupted while waiting for SpawnRobot goal acceptance");
    }
    if (client_node->now() >= goal_accept_deadline)
    {
      return tl::unexpected<std::string>("Timed out waiting for SpawnRobot goal acceptance");
    }
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  rclcpp_action::ClientGoalHandle<SpawnRobot>::SharedPtr goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    return tl::unexpected<std::string>("SpawnRobot goal was rejected by the server");
  }

  // Spin until the result is available, with timeout and shutdown guard.
  std::shared_future<rclcpp_action::ClientGoalHandle<SpawnRobot>::WrappedResult> result_future =
      spawn_client->async_get_result(goal_handle);

  const rclcpp::Time result_deadline = client_node->now() + rclcpp::Duration(kGoalResultTimeout);

  while (result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready)
  {
    if (!rclcpp::ok())
    {
      return tl::unexpected<std::string>("Interrupted while waiting for SpawnRobot result");
    }
    if (client_node->now() >= result_deadline)
    {
      return tl::unexpected<std::string>("Timed out waiting for SpawnRobot result");
    }
    rclcpp::spin_some(client_node);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  const rclcpp_action::ClientGoalHandle<SpawnRobot>::WrappedResult spawn_result = result_future.get();

  if (spawn_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    std::string msg =
        "SpawnRobot action did not succeed (code=" + std::to_string(static_cast<int>(spawn_result.code)) + ")";
    if (!spawn_result.result->message.empty())
    {
      msg += ": " + spawn_result.result->message;
    }
    return tl::unexpected<std::string>(std::move(msg));
  }

  const std::string robot_name = spawn_result.result->indexed_description.name;
  if (robot_name.empty())
  {
    return tl::unexpected<std::string>("SpawnRobot returned an empty robot name");
  }

  return robot_name;
}

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  // --- Parse arguments ---
  tl::expected<SpawnArgs, std::string> args_result = parse_args(argc, argv);
  if (!args_result)
  {
    if (args_result.error() == "__help__")
    {
      rclcpp::shutdown();
      return EXIT_SUCCESS;
    }
    std::cerr << "Error: " << args_result.error() << '\n';
    print_usage(argv[0]);
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const SpawnArgs args = std::move(args_result).value();

  // --- Load robot YAML ---
  tl::expected<stdr_msgs::msg::RobotMsg, std::string> robot_msg_result =
      stdr_parser::load_robot_msg(args.description_path, args.base_dir, args.specs_dir);
  if (!robot_msg_result)
  {
    std::cerr << "Failed to load robot description: " << robot_msg_result.error() << '\n';
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  stdr_msgs::msg::RobotMsg robot_msg = std::move(robot_msg_result).value();

  // Apply CLI pose overrides.
  robot_msg.initial_pose.x = args.x;
  robot_msg.initial_pose.y = args.y;
  robot_msg.initial_pose.theta = args.theta;

  // --- Spawn the robot via the action server ---
  tl::expected<std::string, std::string> name_result = spawn_robot(std::move(robot_msg));
  if (!name_result)
  {
    std::cerr << "Spawn failed: " << name_result.error() << '\n';
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const std::string robot_name = name_result.value();

  // Announce the assigned name so launch files or scripts can capture it.
  std::cout << robot_name << '\n';
  RCLCPP_INFO(rclcpp::get_logger("robot_spawner"), "Robot spawned as '%s'", robot_name.c_str());

  // --- Run the robot node ---
  rclcpp::NodeOptions robot_options;
  robot_options.append_parameter_override("robot_name", robot_name);
  std::shared_ptr<stdr_robot::StdrRobotNode> robot_node = std::make_shared<stdr_robot::StdrRobotNode>(robot_options);

  rclcpp::spin(robot_node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}

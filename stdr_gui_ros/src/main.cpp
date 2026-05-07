#include <stdr_gui/gui_app.hpp>
#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_simulation/types.hpp>

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

namespace
{

void print_usage(const char* program)
{
  std::cerr << "Usage: " << program << " [OPTIONS]\n"
            << "Options:\n"
            << "  --map <path>     Load map YAML file on startup\n"
            << "  --robot <path>   Load robot YAML file on startup\n"
            << "  --x <float>      Robot spawn X position (default: 0)\n"
            << "  --y <float>      Robot spawn Y position (default: 0)\n"
            << "  --theta <float>  Robot spawn heading in radians (default: 0)\n"
            << "  --help           Show this help message\n";
}

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::string map_path;
  std::string robot_path;
  float spawn_x = 0.0f;
  float spawn_y = 0.0f;
  float spawn_theta = 0.0f;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      rclcpp::shutdown();
      return EXIT_SUCCESS;
    }
    if (arg == "--map" && i + 1 < argc)
    {
      map_path = argv[++i];
    }
    else if (arg == "--robot" && i + 1 < argc)
    {
      robot_path = argv[++i];
    }
    else if (arg == "--x" && i + 1 < argc)
    {
      try
      {
        spawn_x = std::stof(argv[++i]);
      }
      catch (const std::exception&)
      {
        std::cerr << "Invalid value for --x: " << argv[i] << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--y" && i + 1 < argc)
    {
      try
      {
        spawn_y = std::stof(argv[++i]);
      }
      catch (const std::exception&)
      {
        std::cerr << "Invalid value for --y: " << argv[i] << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
    }
    else if (arg == "--theta" && i + 1 < argc)
    {
      try
      {
        spawn_theta = std::stof(argv[++i]);
      }
      catch (const std::exception&)
      {
        std::cerr << "Invalid value for --theta: " << argv[i] << '\n';
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
    }
    else if (arg.rfind("--ros-args", 0) == 0 || arg == "--")
    {
      // ROS2 passes --ros-args and related tokens; skip them silently.
      break;
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << '\n';
      print_usage(argv[0]);
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  }

  // Service calls for startup args are not yet wired. Print a note so the user
  // knows the args were parsed but won't have any effect yet.
  if (!map_path.empty() || !robot_path.empty())
  {
    std::cout << "[stdr_gui_ros] Note: --map/--robot startup args accepted but "
                 "service calls are not yet active.\n";
  }

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("stdr_gui");

  // Spin the node on a background thread so ROS2 callbacks are processed while
  // the GUI runs its render loop on the main thread.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor] { executor.spin(); });

  // GuiApp takes unique_ptr ownership; the node is kept alive by the shared_ptr
  // above for the duration of the executor's lifetime.
  std::unique_ptr<stdr_gui_ros::Ros2Backend> backend = std::make_unique<stdr_gui_ros::Ros2Backend>(node);
  stdr_gui::GuiApp app(std::move(backend));

  const tl::expected<void, std::string> result = app.init();
  if (!result)
  {
    std::cerr << "Failed to initialize GUI: " << result.error() << '\n';
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return EXIT_FAILURE;
  }

  const int exit_code = app.run();

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}

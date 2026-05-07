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

// Robot spawning is not done by the GUI in ROS2 mode — robots come up via launch files.
void print_usage(const char* program)
{
  std::cerr << "Usage: " << program << " [OPTIONS]\n"
            << "Options:\n"
            << "  --map <path>     Load map YAML file on startup (calls stdr_server LoadMap)\n"
            << "  --help           Show this help message\n";
}

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  std::string map_path;

  for (int i = 1; i < argc; ++i)
  {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h")
    {
      print_usage(argv[0]);
      rclcpp::shutdown();
      return EXIT_SUCCESS;
    }
    if (arg == "--map")
    {
      if (i + 1 >= argc)
      {
        std::cerr << "--map requires a path argument.\n";
        rclcpp::shutdown();
        return EXIT_FAILURE;
      }
      map_path = argv[++i];
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

  std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("stdr_gui");

  // Spin the node on a background thread so ROS2 callbacks are processed while
  // the GUI runs its render loop on the main thread. The executor must be live
  // before load_map is called: Ros2Backend::load_map relies on the external
  // executor to pump service-discovery and future-completion callbacks.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor] { executor.spin(); });

  // GuiApp takes unique_ptr ownership; the node is kept alive by the shared_ptr
  // above for the duration of the executor's lifetime.
  std::unique_ptr<stdr_gui_ros::Ros2Backend> backend = std::make_unique<stdr_gui_ros::Ros2Backend>(node);

  if (!map_path.empty())
  {
    const tl::expected<void, std::string> load_result = backend->load_map(map_path);
    if (!load_result)
    {
      std::cerr << "Failed to load map: " << load_result.error() << '\n';
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return EXIT_FAILURE;
    }
  }

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

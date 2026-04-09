#include <stdr_gui/gui_app.hpp>
#include "stdr_standalone/standalone_backend.hpp"

#include <stdr_simulation/types.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

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
      spawn_x = std::stof(argv[++i]);
    }
    else if (arg == "--y" && i + 1 < argc)
    {
      spawn_y = std::stof(argv[++i]);
    }
    else if (arg == "--theta" && i + 1 < argc)
    {
      spawn_theta = std::stof(argv[++i]);
    }
    else
    {
      std::cerr << "Unknown argument: " << arg << '\n';
      print_usage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  auto backend = std::make_unique<stdr_standalone::StandaloneBackend>();

  // Load map if specified.
  if (!map_path.empty())
  {
    const tl::expected<void, std::string> map_result = backend->load_map(map_path);
    if (!map_result)
    {
      std::cerr << "Failed to load map: " << map_result.error() << '\n';
      return EXIT_FAILURE;
    }
  }

  // Spawn robot if specified.
  if (!robot_path.empty())
  {
    const stdr_simulation::Pose2D pose{ static_cast<double>(spawn_x), static_cast<double>(spawn_y),
                                        static_cast<double>(spawn_theta) };
    const tl::expected<std::string, std::string> robot_result = backend->spawn_robot(robot_path, pose);
    if (!robot_result)
    {
      std::cerr << "Failed to spawn robot: " << robot_result.error() << '\n';
      return EXIT_FAILURE;
    }
  }

  stdr_gui::GuiApp app(std::move(backend));

  const tl::expected<void, std::string> result = app.init();
  if (!result)
  {
    std::cerr << "Failed to initialize GUI: " << result.error() << '\n';
    return EXIT_FAILURE;
  }

  return app.run();
}

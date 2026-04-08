#include <stdr_gui/gui_app.hpp>
#include "stdr_standalone/standalone_backend.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

int main()
{
  auto backend = std::make_unique<stdr_standalone::StandaloneBackend>();
  stdr_gui::GuiApp app(std::move(backend));

  const auto result = app.init();
  if (!result)
  {
    std::cerr << "Failed to initialize GUI: " << result.error() << '\n';
    return EXIT_FAILURE;
  }

  return app.run();
}

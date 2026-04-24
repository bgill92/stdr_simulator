#include <stdr_gui/plot/registry.hpp>

namespace stdr::plot
{

PlotterRegistry& PlotterRegistry::instance()
{
  // Function-local static guarantees construction on first call, even from
  // pre-main static initializers in plotter TUs.  This avoids the
  // static-initialization-order fiasco that a namespace-scope global would risk.
  static PlotterRegistry registry;
  return registry;
}

std::vector<std::unique_ptr<Plotter>> PlotterRegistry::instantiate_all() const
{
  std::vector<std::unique_ptr<Plotter>> result;
  result.reserve(factories_.size());
  for (const Entry& entry : factories_)
  {
    result.push_back(entry.factory());
  }
  return result;
}

std::size_t PlotterRegistry::size() const
{
  return factories_.size();
}

const std::vector<PlotterRegistry::Entry>& PlotterRegistry::entries() const
{
  return factories_;
}

}  // namespace stdr::plot

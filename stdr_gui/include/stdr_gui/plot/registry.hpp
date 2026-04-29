#pragma once

/** @file Plotter plugin registry and registration macro.
 *
 *  ## How plugins survive dead-stripping
 *
 *  `REGISTER_PLOTTER` produces a file-scope `static bool` initialised from a
 *  call into `PlotterRegistry::instance()`.  When plotters live in a static
 *  library linked with `--gc-sections` (the ament default), object files that
 *  contain no referenced symbols are silently dropped — taking their
 *  registrations with them.
 *
 *  The fix: plotter TUs go into `stdr_gui_plotters` (a STATIC library), and
 *  the consuming binary links it with:
 *
 *    target_link_libraries(my_app PRIVATE
 *        $<LINK_LIBRARY:WHOLE_ARCHIVE,stdr_gui_plotters>)
 *
 *  This pulls every TU into the final link, so `static bool` initializers run
 *  and the registry is populated before `main()`.
 *
 *  ## Singleton construction order
 *
 *  `PlotterRegistry::instance()` returns a function-local `static
 *  PlotterRegistry`.  Function-local statics are initialised on first call
 *  (C++11 §6.7/4), which may happen from a pre-`main` static initializer in
 *  any plotter TU.  This avoids the static-initialisation-order fiasco that
 *  a namespace-scope global singleton would risk. */

#include <stdr_gui/plot/plotter.hpp>

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace stdr::plot
{

/** @brief Central registry of all compiled-in plotter types.
 *
 *  Populated at program start via `REGISTER_PLOTTER` static initializers.
 *  `instantiate_all()` creates one fresh instance of every registered type,
 *  which the `PlotPanel` widget drives each frame. */
class PlotterRegistry
{
public:
  /** Default constructor.  Construct local instances in tests to keep
   *  registrations isolated from the global singleton. */
  PlotterRegistry() = default;

  /** Return the process-wide singleton instance.
   *
   *  Uses function-local static to guarantee construction on first use,
   *  even from pre-`main` static initializers. */
  [[nodiscard]] static PlotterRegistry& instance();

  /** Register a plotter type under @p name.
   *
   *  Called automatically by the `REGISTER_PLOTTER` macro.  Re-registering
   *  the same name overwrites the previous factory — this is intentional to
   *  allow test code to swap in mock types.
   *
   *  @return `true` so the result can initialise a `static bool`. */
  template <typename T>
  [[nodiscard]] bool register_plotter(const std::string& name)
  {
    const auto it = std::ranges::find_if(factories_, [&name](const Entry& e) { return e.name == name; });
    if (it != factories_.end())
    {
      it->factory = [] { return std::make_unique<T>(); };
    }
    else
    {
      factories_.push_back({ name, [] { return std::make_unique<T>(); } });
    }
    return true;
  }

  /** Create one instance of every registered plotter type, in registration
   *  order.  Called once at startup by the `PlotPanel` widget. */
  [[nodiscard]] std::vector<std::unique_ptr<Plotter>> instantiate_all() const;

  /** Return the number of registered plotter types. */
  [[nodiscard]] std::size_t size() const;

  /** @brief One registry entry: name + factory callable.
   *
   *  Exposed so that `PlotPanel` can store per-slot factories for Remove/
   *  reinstantiate without holding a reference to the registry itself. */
  struct Entry
  {
    std::string name;
    std::function<std::unique_ptr<Plotter>()> factory;
  };

  /** Return all registered entries in registration order.
   *
   *  Used by `PlotPanel` to build its slot list at construction time.
   *  Each `Entry::factory` is captured into the slot so the panel can
   *  reinstantiate a single plotter on Remove without querying the registry
   *  again. */
  [[nodiscard]] const std::vector<Entry>& entries() const;

private:
  std::vector<Entry> factories_;
};

}  // namespace stdr::plot

/** @brief Register @p ClassName as a plotter plugin at program start.
 *
 *  Place exactly once at file scope, after the class definition, in any
 *  source file compiled into `stdr_gui_plotters`.  The macro creates an
 *  anonymous-namespace `static bool` whose initializer calls into
 *  `PlotterRegistry::instance()` before `main()`.
 *
 *  @note The consuming target must link `stdr_gui_plotters` with
 *        `$<LINK_LIBRARY:WHOLE_ARCHIVE,stdr_gui_plotters>` to prevent the
 *        linker from dead-stripping the TU and silently skipping registration.
 *
 *  Example:
 *  @code
 *  class MyPlotter : public stdr::plot::Plotter { ... };
 *  REGISTER_PLOTTER(MyPlotter);
 *  @endcode */
#define REGISTER_PLOTTER(ClassName)                                                                                    \
  namespace                                                                                                            \
  {                                                                                                                    \
  static const bool ClassName##_registered_ [[maybe_unused]] =                                                         \
      ::stdr::plot::PlotterRegistry::instance().register_plotter<ClassName>(#ClassName); /* NOLINT(cert-err58-cpp) */  \
  }                                                                                      // namespace

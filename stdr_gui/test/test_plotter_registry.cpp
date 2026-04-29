#include <stdr_gui/plot/plotter.hpp>
#include <stdr_gui/plot/registry.hpp>
#include <stdr_gui/plot/sim_introspection.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace stdr::plot
{
namespace
{

using ::testing::IsEmpty;
using ::testing::SizeIs;

// --- Fake plotter for registry testing ---

class FakePlotter : public Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "FakePlotter";
  }

  void on_sample(SimView& /*sim*/, PlotSink& /*out*/) override
  {
  }

  void on_render(const PlotView& /*data*/) override
  {
  }
};

// --- Registry: direct registration (no macro) ---

TEST(PlotterRegistry, EmptyRegistryInstantiatesNothing)
{
  PlotterRegistry local;
  const std::vector<std::unique_ptr<Plotter>> plotters = local.instantiate_all();
  EXPECT_THAT(plotters, IsEmpty());
  EXPECT_EQ(local.size(), 0u);
}

TEST(PlotterRegistry, RegisterOnePlotterAndInstantiate)
{
  PlotterRegistry local;
  std::ignore = local.register_plotter<FakePlotter>("FakePlotter");

  EXPECT_EQ(local.size(), 1u);

  const std::vector<std::unique_ptr<Plotter>> plotters = local.instantiate_all();
  ASSERT_THAT(plotters, SizeIs(1));
  EXPECT_EQ(plotters[0]->name(), "FakePlotter");
}

TEST(PlotterRegistry, RegisterReturnsTrueForStaticBoolInit)
{
  PlotterRegistry local;
  const bool result = local.register_plotter<FakePlotter>("FakePlotter");
  EXPECT_TRUE(result);
}

TEST(PlotterRegistry, InstantiateAllCreatesUniqueInstances)
{
  PlotterRegistry local;
  std::ignore = local.register_plotter<FakePlotter>("Foo");
  std::ignore = local.register_plotter<FakePlotter>("Bar");

  EXPECT_EQ(local.size(), 2u);

  const std::vector<std::unique_ptr<Plotter>> plotters = local.instantiate_all();
  ASSERT_THAT(plotters, SizeIs(2));

  EXPECT_NE(plotters[0].get(), plotters[1].get());
}

TEST(PlotterRegistry, InstantiateAllProducesFreshInstancesEachCall)
{
  PlotterRegistry local;
  std::ignore = local.register_plotter<FakePlotter>("FakePlotter");

  const std::vector<std::unique_ptr<Plotter>> first = local.instantiate_all();
  const std::vector<std::unique_ptr<Plotter>> second = local.instantiate_all();
  ASSERT_THAT(first, SizeIs(1));
  ASSERT_THAT(second, SizeIs(1));

  EXPECT_NE(first[0].get(), second[0].get());
}

TEST(PlotterRegistry, GlobalSingletonIsStable)
{
  // The function-local static must return the same object on every call.
  EXPECT_EQ(&PlotterRegistry::instance(), &PlotterRegistry::instance());
}

// --- Overwrite behavior ---

class ReplacedPlotter : public Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "Replaced";
  }

  void on_sample(SimView& /*sim*/, PlotSink& /*out*/) override
  {
  }

  void on_render(const PlotView& /*data*/) override
  {
  }
};

class ReplacementPlotter : public Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "Replacement";
  }

  void on_sample(SimView& /*sim*/, PlotSink& /*out*/) override
  {
  }

  void on_render(const PlotView& /*data*/) override
  {
  }
};

TEST(PlotterRegistry, RegisterReplacesExistingPlotter)
{
  PlotterRegistry local;
  std::ignore = local.register_plotter<ReplacedPlotter>("SameName");
  EXPECT_EQ(local.size(), 1u);

  // Re-registering under the same name must overwrite, not append.
  std::ignore = local.register_plotter<ReplacementPlotter>("SameName");
  EXPECT_EQ(local.size(), 1u);

  const std::vector<std::unique_ptr<Plotter>> plotters = local.instantiate_all();
  ASSERT_THAT(plotters, SizeIs(1));
  // The instance must be the replacement type, not the original.
  EXPECT_EQ(plotters[0]->name(), "Replacement");
}

}  // namespace
}  // namespace stdr::plot

// ---------------------------------------------------------------------------
// REGISTER_PLOTTER macro test.
//
// The macro must be used at file scope with a simple (non-namespaced) class
// name.  The plotter class is defined in an anonymous namespace at file scope
// so both the class and the generated static bool are at file scope.
// ---------------------------------------------------------------------------

namespace
{

class MacroTestPlotter : public stdr::plot::Plotter
{
public:
  [[nodiscard]] std::string_view name() const override
  {
    return "MacroTestPlotter";
  }

  void on_sample(stdr::plot::SimView& /*sim*/, stdr::plot::PlotSink& /*out*/) override
  {
  }

  void on_render(const stdr::plot::PlotView& /*data*/) override
  {
  }
};

}  // namespace

REGISTER_PLOTTER(MacroTestPlotter);  // NOLINT(cert-err58-cpp)

namespace stdr::plot
{
namespace
{

TEST(PlotterRegistry, MacroRegistersInGlobalSingleton)
{
  // MacroTestPlotter was registered via REGISTER_PLOTTER above.
  const std::vector<std::unique_ptr<Plotter>> plotters = PlotterRegistry::instance().instantiate_all();
  bool found = false;
  for (const std::unique_ptr<Plotter>& p : plotters)
  {
    if (p->name() == "MacroTestPlotter")
    {
      found = true;
      break;
    }
  }
  EXPECT_TRUE(found) << "MacroTestPlotter not found in global PlotterRegistry after REGISTER_PLOTTER";
}

}  // namespace
}  // namespace stdr::plot

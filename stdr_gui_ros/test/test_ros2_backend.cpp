#include "stdr_gui_ros/ros2_backend.hpp"

#include <stdr_gui/simulator_backend.hpp>

#include <rclcpp/rclcpp.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>

namespace stdr_gui_ros
{
namespace
{

// ─── Fixture ──────────────────────────────────────────────────────────────────

class Ros2BackendTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestSuite()
  {
    rclcpp::shutdown();
  }

  Ros2BackendTest() : node_(std::make_shared<rclcpp::Node>("test_stdr_gui")), backend_(node_)
  {
  }

  std::shared_ptr<rclcpp::Node> node_;
  Ros2Backend backend_;
};

// ─── Construction ─────────────────────────────────────────────────────────────

TEST_F(Ros2BackendTest, DefaultStepDtIsKDefaultStepDt)
{
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kDefaultStepDt);
}

// ─── set_step_dt clamping ─────────────────────────────────────────────────────

TEST_F(Ros2BackendTest, SetStepDtClampsBelowMin)
{
  backend_.set_step_dt(0.0);
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kMinStepDt);
}

TEST_F(Ros2BackendTest, SetStepDtClampsAboveMax)
{
  backend_.set_step_dt(100.0);
  EXPECT_DOUBLE_EQ(backend_.get_step_dt(), stdr_gui::kMaxStepDt);
}

}  // namespace
}  // namespace stdr_gui_ros

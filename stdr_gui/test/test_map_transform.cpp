#include <stdr_gui/map_transform.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

namespace stdr_gui
{
namespace
{

using ::testing::DoubleNear;

constexpr double kTol = 1e-5;

// ---- Default state ----------------------------------------------------------

TEST(MapTransformTest, DefaultState)
{
  const MapTransform t;
  EXPECT_FLOAT_EQ(t.get_zoom(), 1.0f);
  EXPECT_FLOAT_EQ(t.get_offset().x, 0.0f);
  EXPECT_FLOAT_EQ(t.get_offset().y, 0.0f);
}

// ---- World-to-screen identity (resolution=1, origin=(0,0), 10x10) -----------

TEST(MapTransformTest, WorldToScreenIdentity)
{
  MapTransform t;
  t.set_map_info(/*origin_x=*/0.0, /*origin_y=*/0.0, /*resolution=*/1.0,
                 /*width=*/10, /*height=*/10);

  // world (0,0) -> map-pixel (0,0) -> Y-flip -> screen (0, 9)
  const ScreenPoint s0 = t.world_to_screen(0.0, 0.0);
  EXPECT_NEAR(s0.x, 0.0f, 1e-5f);
  EXPECT_NEAR(s0.y, 9.0f, 1e-5f);

  // world (5,5) -> map-pixel (5,5) -> Y-flip -> screen (5, 4)
  const ScreenPoint s1 = t.world_to_screen(5.0, 5.0);
  EXPECT_NEAR(s1.x, 5.0f, 1e-5f);
  EXPECT_NEAR(s1.y, 4.0f, 1e-5f);
}

// ---- World-to-screen with non-unit resolution --------------------------------

TEST(MapTransformTest, WorldToScreenWithResolution)
{
  MapTransform t;
  // resolution=0.05: world x=1.0 -> map-pixel x=20
  t.set_map_info(0.0, 0.0, 0.05, 100, 100);

  const ScreenPoint s = t.world_to_screen(1.0, 0.0);
  EXPECT_NEAR(s.x, 20.0f, 1e-4f);
  // world y=0 -> map-pixel y=0 -> Y-flip -> screen y = 99
  EXPECT_NEAR(s.y, 99.0f, 1e-4f);
}

// ---- World-to-screen with non-zero origin -----------------------------------

TEST(MapTransformTest, WorldToScreenWithOrigin)
{
  MapTransform t;
  // origin=(5,5), resolution=1: world(5,5) -> map-pixel(0,0) -> screen(0,9)
  t.set_map_info(5.0, 5.0, 1.0, 10, 10);

  const ScreenPoint s = t.world_to_screen(5.0, 5.0);
  EXPECT_NEAR(s.x, 0.0f, 1e-5f);
  EXPECT_NEAR(s.y, 9.0f, 1e-5f);
}

// ---- Screen-to-world roundtrip ----------------------------------------------

TEST(MapTransformTest, ScreenToWorldRoundtrip)
{
  MapTransform t;
  t.set_map_info(-2.5, -1.0, 0.05, 200, 150);

  constexpr double kWx = 3.7;
  constexpr double kWy = 1.2;

  const ScreenPoint s = t.world_to_screen(kWx, kWy);
  const auto [rx, ry] = t.screen_to_world(s.x, s.y);

  EXPECT_THAT(rx, DoubleNear(kWx, kTol));
  EXPECT_THAT(ry, DoubleNear(kWy, kTol));
}

// ---- Pan accumulates --------------------------------------------------------

TEST(MapTransformTest, PanAccumulates)
{
  MapTransform t;
  t.pan(10.0f, 20.0f);
  t.pan(5.0f, -3.0f);

  EXPECT_FLOAT_EQ(t.get_offset().x, 15.0f);
  EXPECT_FLOAT_EQ(t.get_offset().y, 17.0f);
}

// ---- Zoom centered on a point -----------------------------------------------

TEST(MapTransformTest, ZoomCenteredOnPoint)
{
  MapTransform t;
  t.set_map_info(0.0, 0.0, 1.0, 100, 100);

  // Choose a pivot point in screen space.
  constexpr float kPivotX = 50.0f;
  constexpr float kPivotY = 50.0f;

  // The world coordinates under the pivot before the zoom.
  const auto [wx_before, wy_before] = t.screen_to_world(kPivotX, kPivotY);

  t.zoom(kPivotX, kPivotY, 2.0f);

  // After zooming, the same screen point should map to the same world point.
  const auto [wx_after, wy_after] = t.screen_to_world(kPivotX, kPivotY);

  EXPECT_THAT(wx_after, DoubleNear(wx_before, kTol));
  EXPECT_THAT(wy_after, DoubleNear(wy_before, kTol));
}

// ---- fit_to_view fits the map -----------------------------------------------

TEST(MapTransformTest, FitToView)
{
  MapTransform t;
  constexpr std::int32_t kMapW = 100;
  constexpr std::int32_t kMapH = 80;
  t.set_map_info(0.0, 0.0, 0.05, kMapW, kMapH);

  constexpr float kVpW = 800.0f;
  constexpr float kVpH = 600.0f;
  t.fit_to_view(kVpW, kVpH);

  // The entire map (in screen pixels) must fit within the viewport.
  const float map_screen_w = static_cast<float>(kMapW) * t.get_zoom();
  const float map_screen_h = static_cast<float>(kMapH) * t.get_zoom();
  EXPECT_LE(map_screen_w, kVpW + 1e-4f);
  EXPECT_LE(map_screen_h, kVpH + 1e-4f);

  // At least one dimension should be flush with the viewport edge.
  const bool fits_width = std::abs(map_screen_w - kVpW) < 1e-3f;
  const bool fits_height = std::abs(map_screen_h - kVpH) < 1e-3f;
  EXPECT_TRUE(fits_width || fits_height);
}

// ---- Zoom clamping ----------------------------------------------------------

TEST(MapTransformTest, ZoomClamp)
{
  MapTransform t;
  t.set_map_info(0.0, 0.0, 1.0, 100, 100);

  // Zooming in far beyond the maximum should be clamped.
  for (int i = 0; i < 200; ++i)
  {
    t.zoom(0.0f, 0.0f, 10.0f);
  }
  EXPECT_LE(t.get_zoom(), MapTransform::kMaxZoom);

  // Zooming out far beyond the minimum should be clamped.
  MapTransform t2;
  t2.set_map_info(0.0, 0.0, 1.0, 100, 100);
  for (int i = 0; i < 200; ++i)
  {
    t2.zoom(0.0f, 0.0f, 0.1f);
  }
  EXPECT_GE(t2.get_zoom(), MapTransform::kMinZoom);
}

// ---- fit_to_view on default-constructed MapTransform is a no-op ---------------

TEST(MapTransformTest, FitToViewNoMapIsNoop)
{
  MapTransform t;
  t.fit_to_view(800.0f, 600.0f);

  EXPECT_FLOAT_EQ(t.get_zoom(), 1.0f);
  EXPECT_FLOAT_EQ(t.get_offset().x, 0.0f);
  EXPECT_FLOAT_EQ(t.get_offset().y, 0.0f);
}

// ---- World-to-screen with negative origin: bottom-left corner ----------------

TEST(MapTransformTest, WorldToScreenNegativeOriginCorner)
{
  MapTransform t;
  // origin=(-5,-3), resolution=0.05, map 200x100 pixels
  t.set_map_info(-5.0, -3.0, 0.05, 200, 100);

  // The bottom-left world corner is at the map origin: wx=-5, wy=-3.
  // map-pixel x = (-5 - (-5)) / 0.05 = 0
  // map-pixel y = (-3 - (-3)) / 0.05 = 0 -> Y-flip -> screen y = (100-1) - 0 = 99
  const ScreenPoint s = t.world_to_screen(-5.0, -3.0);
  EXPECT_NEAR(s.x, 0.0f, 1e-4f);
  EXPECT_NEAR(s.y, 99.0f, 1e-4f);
}

}  // namespace
}  // namespace stdr_gui

#include <stdr_gui/panels/map_panel.hpp>

#include <imgui.h>

#define GLFW_INCLUDE_NONE
#include <GL/gl.h>
#include <GLFW/glfw3.h>

#include <stdr_gui/grid_utils.hpp>
#include <stdr_gui/pose_utils.hpp>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <format>
#include <limits>
#include <string>
#include <vector>

namespace stdr_gui
{

namespace
{

constexpr float kRobotRadius = 0.2f;          // Default footprint radius in world metres.
constexpr float kSelectionThreshold = 15.0f;  // Click distance in pixels to select a robot.
constexpr float kCenterDotRadius = 4.0f;      // Center dot radius in pixels.
constexpr float kMinScreenRadius = 8.0f;      // Minimum robot display radius in pixels.

}  // namespace

MapPanel::~MapPanel()
{
  if (map_texture_ != 0)
  {
    glDeleteTextures(1, &map_texture_);
  }
}

const std::string& MapPanel::selected_robot() const
{
  return selected_robot_;
}

void MapPanel::render(const SimulationSnapshot& snapshot, SimulatorBackend& backend)
{
  ImGui::Begin("Map", nullptr, ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

  const stdr_simulation::OccupancyGrid& grid = snapshot.map;
  if (grid.width > 0 && grid.height > 0)
  {
    update_texture(grid);
    transform_.set_map_info(grid.origin.x, grid.origin.y, grid.resolution, grid.width, grid.height);
    cached_origin_x_ = grid.origin.x;
    cached_origin_y_ = grid.origin.y;
    cached_resolution_ = grid.resolution;
  }

  handle_input(backend);
  render_map_image();
  render_robots(snapshot);
  render_sensor_overlays(snapshot);
  render_environment_sources(snapshot);
  render_map_info_overlay(snapshot);
  render_context_menu(backend, snapshot);

  ImGui::End();
}

void MapPanel::update_texture(const stdr_simulation::OccupancyGrid& grid)
{
  const bool dimensions_changed = (grid.width != cached_map_width_ || grid.height != cached_map_height_);
  const bool data_changed = (grid.data != cached_map_data_);

  if (!dimensions_changed && !data_changed)
  {
    return;
  }

  cached_map_width_ = grid.width;
  cached_map_height_ = grid.height;
  cached_map_data_ = grid.data;

  const std::size_t pixel_count = static_cast<std::size_t>(grid.width) * static_cast<std::size_t>(grid.height);
  std::vector<std::uint8_t> rgba(pixel_count * 4);

  for (std::size_t i = 0; i < pixel_count; ++i)
  {
    const std::int8_t val = (i < grid.data.size()) ? grid.data[i] : -1;
    const RgbaPixel pixel = occupancy_to_rgba(val);
    std::memcpy(rgba.data() + i * 4, pixel.data(), 4);
  }

  if (map_texture_ == 0)
  {
    glGenTextures(1, &map_texture_);
  }

  glBindTexture(GL_TEXTURE_2D, map_texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  if (dimensions_changed)
  {
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, grid.width, grid.height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  }
  else
  {
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, grid.width, grid.height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());
  }

  glBindTexture(GL_TEXTURE_2D, 0);
}

void MapPanel::handle_input(SimulatorBackend& backend)
{
  if (!ImGui::IsWindowFocused() && !ImGui::IsWindowHovered())
  {
    return;
  }

  const ImVec2 window_pos = ImGui::GetWindowPos();
  const ImVec2 mouse_pos = ImGui::GetMousePos();
  const float mx = mouse_pos.x - window_pos.x;
  const float my = mouse_pos.y - window_pos.y;

  // Middle-mouse drag for pan.
  if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
  {
    const ImVec2 delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Middle);
    transform_.pan(delta.x, delta.y);
    ImGui::ResetMouseDragDelta(ImGuiMouseButton_Middle);
  }

  // Mouse wheel for zoom.
  const float wheel = ImGui::GetIO().MouseWheel;
  if (wheel != 0.0f)
  {
    const float factor = (wheel > 0.0f) ? 1.1f : (1.0f / 1.1f);
    transform_.zoom(mx, my, factor);
  }

  // Left-click: select a robot closest to the click.
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
  {
    // Robot selection is done in render_robots via draw list; we just
    // record the click position for distance testing there.  For simplicity,
    // clear selection if no robot is close enough.
    selected_robot_.clear();
  }

  // Capture the right-click position at the moment of the click so the context
  // menu "Teleport here" action can use the original click location rather than
  // the mouse position over the menu item.
  if (ImGui::IsMouseClicked(ImGuiMouseButton_Right))
  {
    context_click_x_ = mouse_pos.x;
    context_click_y_ = mouse_pos.y;
  }
}

void MapPanel::render_map_image()
{
  if (map_texture_ == 0 || cached_map_width_ == 0 || cached_map_height_ == 0)
  {
    return;
  }

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 window_pos = ImGui::GetWindowPos();

  // Map corners in world coordinates: the occupancy grid spans from
  // (origin_x, origin_y) to (origin_x + width*resolution, origin_y + height*resolution).
  // In screen space, after Y-flip, the world top-left becomes screen top and world bottom becomes screen bottom.
  const double world_right = cached_origin_x_ + cached_map_width_ * cached_resolution_;
  const double world_top = cached_origin_y_ + cached_map_height_ * cached_resolution_;

  // Screen top-left: world top-left corner.
  const ScreenPoint tl = transform_.world_to_screen(cached_origin_x_, world_top);
  // Screen bottom-right: world bottom-right corner.
  const ScreenPoint br = transform_.world_to_screen(world_right, cached_origin_y_);

  const ImVec2 p_min{ window_pos.x + tl.x, window_pos.y + tl.y };
  const ImVec2 p_max{ window_pos.x + br.x, window_pos.y + br.y };

  // OpenGL texture coordinates: (0,0)=bottom-left, (1,1)=top-right.
  // ImGui screen: y increases downward, ROS map: y increases upward, so we
  // flip the UV vertically.
  draw_list->AddImage(ImTextureRef(static_cast<ImTextureID>(map_texture_)), p_min, p_max,
                      ImVec2(0.0f, 1.0f),   // UV top-left: bottom of texture.
                      ImVec2(1.0f, 0.0f));  // UV bottom-right: top of texture.
}

void MapPanel::render_robots(const SimulationSnapshot& snapshot)
{
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 window_pos = ImGui::GetWindowPos();
  const ImVec2 mouse_pos = ImGui::GetMousePos();

  float best_dist = std::numeric_limits<float>::max();

  for (const stdr_simulation::world::RobotState& robot : snapshot.robots)
  {
    const ScreenPoint sp = transform_.world_to_screen(robot.pose.x, robot.pose.y);
    const ImVec2 center{ window_pos.x + sp.x, window_pos.y + sp.y };

    // Determine display radius: use config radius if available, else default.
    const float world_radius =
        (robot.config.footprint.radius > 0.0) ? static_cast<float>(robot.config.footprint.radius) : kRobotRadius;
    float screen_radius = std::max(
        world_radius / static_cast<float>(transform_.get_resolution()) * transform_.get_zoom(), kMinScreenRadius);

    const bool is_selected = (robot.name == selected_robot_);
    const ImU32 fill_color = is_selected ? IM_COL32(255, 200, 0, 180) : IM_COL32(0, 120, 255, 180);

    const bool has_polygon = !robot.config.footprint.points.empty();

    if (has_polygon)
    {
      // Build screen-space polygon vertices by rotating each local-frame point
      // by the robot heading and translating to world position.
      std::vector<ImVec2> poly_screen;
      poly_screen.reserve(robot.config.footprint.points.size());

      const double cos_theta = std::cos(robot.pose.theta);
      const double sin_theta = std::sin(robot.pose.theta);

      float max_screen_dist = 0.0f;

      for (const stdr_simulation::Point2D& pt : robot.config.footprint.points)
      {
        // Rotate point by robot heading, then translate to world position.
        const double wx = robot.pose.x + pt.x * cos_theta - pt.y * sin_theta;
        const double wy = robot.pose.y + pt.x * sin_theta + pt.y * cos_theta;

        const ScreenPoint vsp = transform_.world_to_screen(wx, wy);
        const ImVec2 vertex{ window_pos.x + vsp.x, window_pos.y + vsp.y };
        poly_screen.push_back(vertex);

        // Track max distance from center for arrow length and selection.
        const float dx = vertex.x - center.x;
        const float dy = vertex.y - center.y;
        max_screen_dist = std::max(max_screen_dist, std::sqrt(dx * dx + dy * dy));
      }

      screen_radius = std::max(max_screen_dist, kMinScreenRadius);

      // Draw filled polygon and outline.
      // Use concave fill because the trin_bot D-shaped polygon is non-convex.
      draw_list->AddConcavePolyFilled(poly_screen.data(), static_cast<int>(poly_screen.size()), fill_color);
      // Draw outline using path API for reliable rendering over the concave fill.
      for (int i = 0; i < static_cast<int>(poly_screen.size()); ++i)
      {
        draw_list->PathLineTo(poly_screen[static_cast<size_t>(i)]);
      }
      draw_list->PathStroke(IM_COL32(0, 200, 0, 255), ImDrawFlags_Closed, 3.0f);
    }
    else
    {
      // Circular footprint.
      draw_list->AddCircleFilled(center, screen_radius, fill_color);
      draw_list->AddCircle(center, screen_radius, IM_COL32(0, 200, 0, 255), 32, 1.5f);
    }

    // Center dot for visibility at any zoom level.
    draw_list->AddCircleFilled(center, kCenterDotRadius, IM_COL32(255, 0, 0, 255));

    // Draw orientation arrow from center to the footprint edge.
    const float arrow_screen = screen_radius;
    const ImVec2 tip{ center.x + arrow_screen * static_cast<float>(std::cos(robot.pose.theta)),
                      center.y - arrow_screen * static_cast<float>(std::sin(robot.pose.theta)) };
    draw_list->AddLine(center, tip, IM_COL32(255, 0, 0, 255), 2.5f);

    // Robot name label.
    draw_list->AddText(ImVec2(center.x + screen_radius + 3.0f, center.y - 8.0f), IM_COL32(255, 255, 255, 255),
                       robot.name.c_str());

    // Left-click selection: pick the robot closest to the click within threshold.
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
    {
      const float dx = mouse_pos.x - center.x;
      const float dy = mouse_pos.y - center.y;
      const float dist = std::sqrt(dx * dx + dy * dy);
      const float effective_threshold = std::max(kSelectionThreshold, screen_radius);
      if (dist < effective_threshold && dist < best_dist)
      {
        best_dist = dist;
        selected_robot_ = robot.name;
      }
    }
  }
}

void MapPanel::render_sensor_overlays(const SimulationSnapshot& snapshot)
{
  if (selected_robot_.empty())
  {
    return;
  }

  const auto robot_it = std::ranges::find_if(snapshot.robots, [&](const stdr_simulation::world::RobotState& r) {
    return r.name == selected_robot_;
  });
  if (robot_it == snapshot.robots.end())
  {
    return;
  }

  const auto data_it = snapshot.sensor_data.find(selected_robot_);
  if (data_it == snapshot.sensor_data.end())
  {
    return;
  }

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 window_pos = ImGui::GetWindowPos();

  const stdr_simulation::RobotSensorData& data = data_it->second;
  const stdr_simulation::world::RobotState& robot = *robot_it;

  // Draw laser scan rays for each laser sensor.
  for (std::size_t li = 0; li < data.laser_scans.size(); ++li)
  {
    const stdr_simulation::LaserScan& scan = data.laser_scans[li];
    if (scan.ranges.empty())
    {
      continue;
    }

    // Sensor pose is relative to robot.
    const stdr_simulation::Pose2D sensor_pose =
        (li < robot.config.laser_sensors.size()) ? robot.config.laser_sensors[li].pose : stdr_simulation::Pose2D{};
    const stdr_simulation::Pose2D sensor_world = transform_to_world(robot.pose, sensor_pose);

    const ScreenPoint sensor_sp = transform_.world_to_screen(sensor_world.x, sensor_world.y);
    const ImVec2 sensor_screen{ window_pos.x + sensor_sp.x, window_pos.y + sensor_sp.y };

    for (std::size_t i = 0; i < scan.ranges.size(); ++i)
    {
      const double angle = sensor_world.theta + scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double range = static_cast<double>(scan.ranges[i]);
      const double end_wx = sensor_world.x + range * std::cos(angle);
      const double end_wy = sensor_world.y + range * std::sin(angle);

      const ScreenPoint end_sp = transform_.world_to_screen(end_wx, end_wy);
      const ImVec2 end_screen{ window_pos.x + end_sp.x, window_pos.y + end_sp.y };

      draw_list->AddLine(sensor_screen, end_screen, IM_COL32(255, 50, 50, 80), 1.0f);
    }
  }

  // Draw sonar cones for each sonar sensor.
  for (std::size_t si = 0; si < data.sonar_scans.size(); ++si)
  {
    const stdr_simulation::SonarScan& scan = data.sonar_scans[si];
    if (si >= robot.config.sonar_sensors.size())
    {
      continue;
    }

    const stdr_simulation::SonarConfig& cfg = robot.config.sonar_sensors[si];
    const stdr_simulation::Pose2D sensor_world = transform_to_world(robot.pose, cfg.pose);

    const ScreenPoint sensor_sp = transform_.world_to_screen(sensor_world.x, sensor_world.y);
    const ImVec2 sensor_screen{ window_pos.x + sensor_sp.x, window_pos.y + sensor_sp.y };

    const double range = scan.range;
    const double half_cone = cfg.cone_angle * 0.5;

    const double left_angle = sensor_world.theta + half_cone;
    const double right_angle = sensor_world.theta - half_cone;

    const double left_wx = sensor_world.x + range * std::cos(left_angle);
    const double left_wy = sensor_world.y + range * std::sin(left_angle);
    const double right_wx = sensor_world.x + range * std::cos(right_angle);
    const double right_wy = sensor_world.y + range * std::sin(right_angle);

    const ScreenPoint left_sp = transform_.world_to_screen(left_wx, left_wy);
    const ScreenPoint right_sp = transform_.world_to_screen(right_wx, right_wy);

    const ImVec2 left_screen{ window_pos.x + left_sp.x, window_pos.y + left_sp.y };
    const ImVec2 right_screen{ window_pos.x + right_sp.x, window_pos.y + right_sp.y };

    draw_list->AddLine(sensor_screen, left_screen, IM_COL32(50, 200, 255, 120), 1.5f);
    draw_list->AddLine(sensor_screen, right_screen, IM_COL32(50, 200, 255, 120), 1.5f);
    draw_list->AddLine(left_screen, right_screen, IM_COL32(50, 200, 255, 120), 1.5f);
  }
}

void MapPanel::render_environment_sources(const SimulationSnapshot& snapshot)
{
  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  const ImVec2 window_pos = ImGui::GetWindowPos();
  constexpr float kSourceRadius = 6.0f;

  // RFID tags: green.
  for (const stdr_simulation::RfidTag& tag : snapshot.rfid_tags)
  {
    const ScreenPoint sp = transform_.world_to_screen(tag.pose.x, tag.pose.y);
    const ImVec2 center{ window_pos.x + sp.x, window_pos.y + sp.y };
    draw_list->AddCircleFilled(center, kSourceRadius, IM_COL32(0, 220, 50, 200));
    draw_list->AddText(ImVec2(center.x + kSourceRadius + 2.0f, center.y - 6.0f), IM_COL32(0, 220, 50, 255),
                       tag.tag_id.c_str());
  }

  // CO2 sources: red.
  for (const stdr_simulation::CO2Source& src : snapshot.co2_sources)
  {
    const ScreenPoint sp = transform_.world_to_screen(src.pose.x, src.pose.y);
    const ImVec2 center{ window_pos.x + sp.x, window_pos.y + sp.y };
    draw_list->AddCircleFilled(center, kSourceRadius, IM_COL32(220, 50, 50, 200));
    draw_list->AddText(ImVec2(center.x + kSourceRadius + 2.0f, center.y - 6.0f), IM_COL32(220, 50, 50, 255),
                       src.id.c_str());
  }

  // Thermal sources: orange.
  for (const stdr_simulation::ThermalSource& src : snapshot.thermal_sources)
  {
    const ScreenPoint sp = transform_.world_to_screen(src.pose.x, src.pose.y);
    const ImVec2 center{ window_pos.x + sp.x, window_pos.y + sp.y };
    draw_list->AddCircleFilled(center, kSourceRadius, IM_COL32(255, 140, 0, 200));
    draw_list->AddText(ImVec2(center.x + kSourceRadius + 2.0f, center.y - 6.0f), IM_COL32(255, 140, 0, 255),
                       src.id.c_str());
  }

  // Sound sources: blue.
  for (const stdr_simulation::SoundSource& src : snapshot.sound_sources)
  {
    const ScreenPoint sp = transform_.world_to_screen(src.pose.x, src.pose.y);
    const ImVec2 center{ window_pos.x + sp.x, window_pos.y + sp.y };
    draw_list->AddCircleFilled(center, kSourceRadius, IM_COL32(80, 120, 255, 200));
    draw_list->AddText(ImVec2(center.x + kSourceRadius + 2.0f, center.y - 6.0f), IM_COL32(80, 120, 255, 255),
                       src.id.c_str());
  }
}

void MapPanel::render_map_info_overlay(const SimulationSnapshot& snapshot)
{
  const stdr_simulation::OccupancyGrid& grid = snapshot.map;
  if (grid.width <= 0 || grid.height <= 0)
  {
    return;
  }

  // Semi-transparent overlay in the bottom-left corner of the map window.
  const ImVec2 window_pos = ImGui::GetWindowPos();
  const ImVec2 window_size = ImGui::GetWindowSize();

  constexpr float kPadding = 8.0f;
  constexpr float kOverlayWidth = 220.0f;
  constexpr float kOverlayHeight = 80.0f;

  const ImVec2 overlay_pos{ window_pos.x + kPadding, window_pos.y + window_size.y - kOverlayHeight - kPadding };
  const ImVec2 overlay_end{ overlay_pos.x + kOverlayWidth, overlay_pos.y + kOverlayHeight };

  ImDrawList* draw_list = ImGui::GetWindowDrawList();
  draw_list->AddRectFilled(overlay_pos, overlay_end, IM_COL32(0, 0, 0, 180), 4.0f);

  const float world_w = static_cast<float>(grid.width) * static_cast<float>(grid.resolution);
  const float world_h = static_cast<float>(grid.height) * static_cast<float>(grid.resolution);

  const float text_x = overlay_pos.x + 6.0f;
  float text_y = overlay_pos.y + 4.0f;
  constexpr float kLineHeight = 15.0f;

  const ImU32 text_color = IM_COL32(220, 220, 220, 255);
  const ImU32 label_color = IM_COL32(160, 160, 160, 255);

  if (!snapshot.map_name.empty())
  {
    draw_list->AddText(ImVec2(text_x, text_y), label_color, snapshot.map_name.c_str());
    text_y += kLineHeight;
  }

  // Resolution line.
  const std::string res_text = std::format("Resolution: {:.4f} m/px", grid.resolution);
  draw_list->AddText(ImVec2(text_x, text_y), text_color, res_text.c_str());
  text_y += kLineHeight;

  // Dimensions in pixels.
  const std::string dim_text = std::format("Size: {}x{} px", grid.width, grid.height);
  draw_list->AddText(ImVec2(text_x, text_y), text_color, dim_text.c_str());
  text_y += kLineHeight;

  // Dimensions in meters.
  const std::string world_text = std::format("World: {:.2f}x{:.2f} m", world_w, world_h);
  draw_list->AddText(ImVec2(text_x, text_y), text_color, world_text.c_str());
}

void MapPanel::render_context_menu(SimulatorBackend& backend, const SimulationSnapshot& snapshot)
{
  if (ImGui::BeginPopupContextWindow("##MapContextMenu"))
  {
    if (!selected_robot_.empty())
    {
      ImGui::Text("Robot: %s", selected_robot_.c_str());
      ImGui::Separator();

      if (ImGui::MenuItem("Delete robot"))
      {
        backend.delete_robot(selected_robot_);
        selected_robot_.clear();
      }

      // Teleport: use the right-click position captured in handle_input so the
      // target is the map location where the user clicked, not the menu item.
      if (ImGui::MenuItem("Teleport here"))
      {
        const ImVec2 window_pos = ImGui::GetWindowPos();
        const auto [wx, wy] =
            transform_.screen_to_world(context_click_x_ - window_pos.x, context_click_y_ - window_pos.y);

        // Find current theta to preserve orientation.
        double theta = 0.0;
        for (const stdr_simulation::world::RobotState& r : snapshot.robots)
        {
          if (r.name == selected_robot_)
          {
            theta = r.pose.theta;
            break;
          }
        }
        backend.set_robot_pose(selected_robot_, stdr_simulation::Pose2D{ wx, wy, theta });
      }
    }
    else
    {
      ImGui::TextDisabled("No robot selected");
    }

    ImGui::EndPopup();
  }
}

}  // namespace stdr_gui

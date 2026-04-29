#include <stdr_gui/plot/plotter.hpp>

namespace stdr::plot
{

// ---------------------------------------------------------------------------
// PlotSink
// ---------------------------------------------------------------------------

void PlotSink::scalar(std::string_view name, double timestamp, double value)
{
  std::vector<TimedScalar>& data = channel<TimedScalar>(name, kDefaultCapacity);
  data.push_back(TimedScalar{ timestamp, value });
}

void PlotSink::point(std::string_view name, double x, double y)
{
  std::vector<Point2>& data = channel<Point2>(name, kDefaultCapacity);
  data.push_back(Point2{ x, y });
}

void PlotSink::vector(std::string_view name, std::span<const double> xs, std::span<const double> ys)
{
  std::vector<Point2>& data = channel<Point2>(name, kDefaultCapacity);
  const std::size_t count = std::min(xs.size(), ys.size());
  data.reserve(data.size() + count);
  for (std::size_t i = 0; i < count; ++i)
  {
    data.push_back(Point2{ xs[i], ys[i] });
  }
}

// ---------------------------------------------------------------------------
// PlotView
// ---------------------------------------------------------------------------

std::span<const TimedScalar> PlotView::scalar(std::string_view name) const
{
  const detail::TypedChannel<TimedScalar>* ch = sink_.find_channel<TimedScalar>(name);
  if (ch == nullptr)
  {
    return {};
  }
  return std::span<const TimedScalar>{ ch->data.data(), ch->data.size() };
}

std::span<const Point2> PlotView::points(std::string_view name) const
{
  const detail::TypedChannel<Point2>* ch = sink_.find_channel<Point2>(name);
  if (ch == nullptr)
  {
    return {};
  }
  return std::span<const Point2>{ ch->data.data(), ch->data.size() };
}

}  // namespace stdr::plot

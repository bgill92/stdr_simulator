#include <stdr_gui/plot/plotter.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <span>
#include <string>
#include <vector>

namespace stdr::plot
{
namespace
{

using ::testing::DoubleEq;
using ::testing::IsEmpty;
using ::testing::SizeIs;

// --- PlotSink: lazy ring allocation ---

TEST(PlotSink, ScalarLazilyAllocatesChannelOnFirstCall)
{
  PlotSink sink;
  // No channel exists yet — calling scalar() must not crash and must create it.
  sink.scalar("my_channel", 1.0, 42.0);

  const detail::TypedChannel<TimedScalar>* ch = sink.find_channel<TimedScalar>("my_channel");
  ASSERT_NE(ch, nullptr);
  ASSERT_THAT(ch->data, SizeIs(1));
}

TEST(PlotSink, ScalarAppendsToExistingChannel)
{
  PlotSink sink;
  sink.scalar("ch", 0.1, 1.0);
  sink.scalar("ch", 0.2, 2.0);
  sink.scalar("ch", 0.3, 3.0);

  const detail::TypedChannel<TimedScalar>* ch = sink.find_channel<TimedScalar>("ch");
  ASSERT_NE(ch, nullptr);
  ASSERT_THAT(ch->data, SizeIs(3));
  EXPECT_DOUBLE_EQ(ch->data[0].t, 0.1);
  EXPECT_DOUBLE_EQ(ch->data[0].v, 1.0);
  EXPECT_DOUBLE_EQ(ch->data[2].t, 0.3);
  EXPECT_DOUBLE_EQ(ch->data[2].v, 3.0);
}

TEST(PlotSink, PointLazilyAllocatesChannelOnFirstCall)
{
  PlotSink sink;
  sink.point("pts", 3.0, 4.0);

  const detail::TypedChannel<Point2>* ch = sink.find_channel<Point2>("pts");
  ASSERT_NE(ch, nullptr);
  ASSERT_THAT(ch->data, SizeIs(1));
  EXPECT_DOUBLE_EQ(ch->data[0].x, 3.0);
  EXPECT_DOUBLE_EQ(ch->data[0].y, 4.0);
}

TEST(PlotSink, VectorAppendsParallelArrays)
{
  PlotSink sink;
  const std::vector<double> xs = { 1.0, 2.0, 3.0 };
  const std::vector<double> ys = { 4.0, 5.0, 6.0 };
  sink.vector("path", xs, ys);

  const detail::TypedChannel<Point2>* ch = sink.find_channel<Point2>("path");
  ASSERT_NE(ch, nullptr);
  ASSERT_THAT(ch->data, SizeIs(3));
  EXPECT_DOUBLE_EQ(ch->data[1].x, 2.0);
  EXPECT_DOUBLE_EQ(ch->data[1].y, 5.0);
}

TEST(PlotSink, VectorTruncatesToShorterSpan)
{
  PlotSink sink;
  const std::vector<double> xs = { 1.0, 2.0, 3.0 };
  const std::vector<double> ys = { 4.0, 5.0 };  // shorter
  sink.vector("path", xs, ys);

  const detail::TypedChannel<Point2>* ch = sink.find_channel<Point2>("path");
  ASSERT_NE(ch, nullptr);
  // Must not read beyond the shorter span.
  EXPECT_THAT(ch->data, SizeIs(2));
}

TEST(PlotSink, ChannelTemplateAllocatesTypedStorage)
{
  PlotSink sink;
  std::vector<int>& data = sink.channel<int>("ints", 64);
  data.push_back(10);
  data.push_back(20);

  const detail::TypedChannel<int>* ch = sink.find_channel<int>("ints");
  ASSERT_NE(ch, nullptr);
  ASSERT_THAT(ch->data, SizeIs(2));
  EXPECT_EQ(ch->data[0], 10);
  EXPECT_EQ(ch->data[1], 20);
}

TEST(PlotSink, FindChannelReturnsNullptrForMissingChannel)
{
  const PlotSink sink;
  EXPECT_EQ(sink.find_channel<TimedScalar>("does_not_exist"), nullptr);
}

TEST(PlotSink, MultipleChannelTypesCoexist)
{
  PlotSink sink;
  sink.scalar("scalars", 1.0, 99.0);
  sink.point("points", 2.0, 3.0);

  EXPECT_NE(sink.find_channel<TimedScalar>("scalars"), nullptr);
  EXPECT_NE(sink.find_channel<Point2>("points"), nullptr);
  // Cross-type lookup must not find the wrong channel.
  EXPECT_EQ(sink.find_channel<Point2>("scalars"), nullptr);
  EXPECT_EQ(sink.find_channel<TimedScalar>("points"), nullptr);
}

// --- PlotView: read from PlotSink ---

TEST(PlotView, ScalarReturnsEmptySpanForMissingChannel)
{
  const PlotSink sink;
  const PlotView view(sink);
  EXPECT_THAT(view.scalar("missing"), IsEmpty());
}

TEST(PlotView, ScalarReturnsAllWrittenValues)
{
  PlotSink sink;
  sink.scalar("ch", 0.5, 10.0);
  sink.scalar("ch", 1.0, 20.0);

  const PlotView view(sink);
  const std::span<const TimedScalar> span = view.scalar("ch");
  ASSERT_THAT(span, SizeIs(2));
  EXPECT_DOUBLE_EQ(span[0].t, 0.5);
  EXPECT_DOUBLE_EQ(span[0].v, 10.0);
  EXPECT_DOUBLE_EQ(span[1].t, 1.0);
  EXPECT_DOUBLE_EQ(span[1].v, 20.0);
}

TEST(PlotView, PointsReturnsEmptySpanForMissingChannel)
{
  const PlotSink sink;
  const PlotView view(sink);
  EXPECT_THAT(view.points("missing"), IsEmpty());
}

TEST(PlotView, PointsReturnsAllWrittenValues)
{
  PlotSink sink;
  sink.point("pts", 1.0, 2.0);
  sink.point("pts", 3.0, 4.0);

  const PlotView view(sink);
  const std::span<const Point2> span = view.points("pts");
  ASSERT_THAT(span, SizeIs(2));
  EXPECT_DOUBLE_EQ(span[0].x, 1.0);
  EXPECT_DOUBLE_EQ(span[0].y, 2.0);
  EXPECT_DOUBLE_EQ(span[1].x, 3.0);
  EXPECT_DOUBLE_EQ(span[1].y, 4.0);
}

TEST(PlotView, ValuesTemplateReturnsEmptySpanForMissingChannel)
{
  const PlotSink sink;
  const PlotView view(sink);
  EXPECT_THAT(view.values<int>("missing"), IsEmpty());
}

TEST(PlotView, ValuesTemplateReturnsTypedData)
{
  PlotSink sink;
  sink.channel<int>("ints") = { 7, 8, 9 };

  const PlotView view(sink);
  const std::span<const int> span = view.values<int>("ints");
  ASSERT_THAT(span, SizeIs(3));
  EXPECT_EQ(span[0], 7);
  EXPECT_EQ(span[1], 8);
  EXPECT_EQ(span[2], 9);
}

TEST(PlotView, SpanDataMatchesAfterAdditionalWrites)
{
  PlotSink sink;
  sink.scalar("ch", 0.0, 0.0);

  // Write more data; the PlotView re-queries the underlying vector each call.
  sink.scalar("ch", 1.0, 1.0);

  const PlotView view(sink);
  const std::span<const TimedScalar> span = view.scalar("ch");
  EXPECT_THAT(span, SizeIs(2));
}

}  // namespace
}  // namespace stdr::plot

#include <stdr_gui/panels/toolbar.hpp>

#include <gtest/gtest.h>

namespace stdr_gui
{

TEST(FormatElapsedTime, ZeroSeconds)
{
  EXPECT_EQ(format_elapsed_time(0.0), "00:00:00.00");
}

TEST(FormatElapsedTime, OneAndHalfSeconds)
{
  EXPECT_EQ(format_elapsed_time(1.5), "00:00:01.50");
}

TEST(FormatElapsedTime, EightyThreePointFortyFiveSeconds)
{
  EXPECT_EQ(format_elapsed_time(83.45), "00:01:23.45");
}

TEST(FormatElapsedTime, OneHourOneMinuteOneSecond)
{
  EXPECT_EQ(format_elapsed_time(3661.0), "01:01:01.00");
}

TEST(FormatElapsedTime, NegativeInputClampsToZero)
{
  EXPECT_EQ(format_elapsed_time(-5.0), "00:00:00.00");
}

}  // namespace stdr_gui

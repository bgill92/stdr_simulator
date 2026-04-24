#include <stdr_simulation/plot_data/latest_snapshot.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <string>

namespace stdr::plot_data
{
namespace
{

using ::testing::Eq;
using ::testing::IsNull;
using ::testing::NotNull;

// ---------------------------------------------------------------------------
// Basic publish / read
// ---------------------------------------------------------------------------

TEST(LatestSnapshotTest, ReadWithNoPublishReturnsNullptr)
{
  LatestSnapshot<int> cell;
  EXPECT_THAT(cell.read(), IsNull());
}

TEST(LatestSnapshotTest, ReadAfterPublishReturnsValue)
{
  LatestSnapshot<int> cell;
  cell.publish(std::make_shared<const int>(42));

  const std::shared_ptr<const int> got = cell.read();
  ASSERT_THAT(got, NotNull());
  EXPECT_THAT(*got, Eq(42));
}

// ---------------------------------------------------------------------------
// Multiple publishes — reader always gets latest
// ---------------------------------------------------------------------------

TEST(LatestSnapshotTest, MultiplePublishesReturnLatest)
{
  LatestSnapshot<int> cell;
  cell.publish(std::make_shared<const int>(1));
  cell.publish(std::make_shared<const int>(2));
  cell.publish(std::make_shared<const int>(3));

  const std::shared_ptr<const int> got = cell.read();
  ASSERT_THAT(got, NotNull());
  EXPECT_THAT(*got, Eq(3));
}

// ---------------------------------------------------------------------------
// Old snapshot remains alive while caller holds shared_ptr
// ---------------------------------------------------------------------------

TEST(LatestSnapshotTest, HeldSharedPtrKeepsOldCopyAliveAfterNewPublish)
{
  LatestSnapshot<std::string> cell;
  cell.publish(std::make_shared<const std::string>("first"));

  // Consumer holds a reference to the first value.
  const std::shared_ptr<const std::string> held = cell.read();
  ASSERT_THAT(held, NotNull());

  // Producer publishes a new value.
  cell.publish(std::make_shared<const std::string>("second"));

  // Consumer's old pointer is still alive and points to "first".
  EXPECT_THAT(*held, Eq(std::string{ "first" }));

  // A fresh read returns the new value.
  const std::shared_ptr<const std::string> fresh = cell.read();
  ASSERT_THAT(fresh, NotNull());
  EXPECT_THAT(*fresh, Eq(std::string{ "second" }));
}

// ---------------------------------------------------------------------------
// Struct payload — verify no data corruption
// ---------------------------------------------------------------------------

struct SamplePayload
{
  double x{ 0.0 };
  double y{ 0.0 };
  int id{ 0 };
};

TEST(LatestSnapshotTest, StructPayloadPreservesAllFields)
{
  LatestSnapshot<SamplePayload> cell;
  cell.publish(std::make_shared<const SamplePayload>(SamplePayload{ 1.5, 2.5, 7 }));

  const std::shared_ptr<const SamplePayload> got = cell.read();
  ASSERT_THAT(got, NotNull());
  EXPECT_THAT(got->x, Eq(1.5));
  EXPECT_THAT(got->y, Eq(2.5));
  EXPECT_THAT(got->id, Eq(7));
}

// ---------------------------------------------------------------------------
// Repeated reads return same value (idempotent)
// ---------------------------------------------------------------------------

TEST(LatestSnapshotTest, RepeatedReadWithoutNewPublishReturnsSameValue)
{
  LatestSnapshot<int> cell;
  cell.publish(std::make_shared<const int>(99));

  const std::shared_ptr<const int> first_read = cell.read();
  const std::shared_ptr<const int> second_read = cell.read();

  ASSERT_THAT(first_read, NotNull());
  ASSERT_THAT(second_read, NotNull());
  EXPECT_THAT(*first_read, Eq(99));
  EXPECT_THAT(*second_read, Eq(99));
}

// ---------------------------------------------------------------------------
// Publish nullptr explicitly clears the cell
// ---------------------------------------------------------------------------

TEST(LatestSnapshotTest, PublishNullptrClearsTheCell)
{
  LatestSnapshot<int> cell;
  cell.publish(std::make_shared<const int>(5));
  cell.publish(nullptr);

  EXPECT_THAT(cell.read(), IsNull());
}

}  // namespace
}  // namespace stdr::plot_data

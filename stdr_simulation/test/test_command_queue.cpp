#include <stdr_simulation/plot_data/command_queue.hpp>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <thread>
#include <vector>

namespace stdr::plot_data
{
namespace
{

using ::testing::IsEmpty;
using ::testing::SizeIs;

// --- Empty queue ---

TEST(CommandQueue, DrainOnEmptyReturnsEmptyVector)
{
  CommandQueue queue;
  const std::vector<Command> result = queue.drain();
  EXPECT_THAT(result, IsEmpty());
}

// --- Single command ---

TEST(CommandQueue, SinglePushDrainReturnsCommand)
{
  CommandQueue queue;
  Command cmd;
  cmd.kind = Command::Kind::Velocity;
  cmd.robot = "robot0";
  cmd.twist = stdr_simulation::Twist2D{ .linear_x = 1.0, .linear_y = 0.0, .angular_z = 0.5 };

  queue.push(cmd);

  const std::vector<Command> result = queue.drain();
  ASSERT_THAT(result, SizeIs(1));
  EXPECT_EQ(result[0].kind, Command::Kind::Velocity);
  EXPECT_EQ(result[0].robot, "robot0");
  EXPECT_DOUBLE_EQ(result[0].twist.linear_x, 1.0);
  EXPECT_DOUBLE_EQ(result[0].twist.angular_z, 0.5);
}

// --- Multiple commands in FIFO order ---

TEST(CommandQueue, MultipleCommandsDrainInFifoOrder)
{
  CommandQueue queue;

  Command cmd0;
  cmd0.kind = Command::Kind::Velocity;
  cmd0.robot = "robot0";
  cmd0.twist = stdr_simulation::Twist2D{ .linear_x = 1.0, .linear_y = 0.0, .angular_z = 0.0 };

  Command cmd1;
  cmd1.kind = Command::Kind::Teleport;
  cmd1.robot = "robot1";
  cmd1.pose = stdr_simulation::Pose2D{ .x = 2.0, .y = 3.0, .theta = 0.5 };

  Command cmd2;
  cmd2.kind = Command::Kind::Pause;

  queue.push(cmd0);
  queue.push(cmd1);
  queue.push(cmd2);

  const std::vector<Command> result = queue.drain();
  ASSERT_THAT(result, SizeIs(3));
  EXPECT_EQ(result[0].kind, Command::Kind::Velocity);
  EXPECT_EQ(result[0].robot, "robot0");
  EXPECT_EQ(result[1].kind, Command::Kind::Teleport);
  EXPECT_EQ(result[1].robot, "robot1");
  EXPECT_DOUBLE_EQ(result[1].pose.x, 2.0);
  EXPECT_EQ(result[2].kind, Command::Kind::Pause);
}

// --- Drain empties the queue ---

TEST(CommandQueue, DrainEmptiesQueueForNextDrain)
{
  CommandQueue queue;
  Command cmd;
  cmd.kind = Command::Kind::Resume;
  queue.push(cmd);

  std::ignore = queue.drain();

  const std::vector<Command> second = queue.drain();
  EXPECT_THAT(second, IsEmpty());
}

// --- Concurrent producers ---

TEST(CommandQueue, ConcurrentProducersAllCommandsDelivered)
{
  // N producers each push M commands; the sum of drained commands after all
  // threads join must equal N*M with no torn (partially written) entries.
  constexpr int kProducers = 8;
  constexpr int kCommandsPerProducer = 500;

  CommandQueue queue;

  std::vector<std::thread> threads;
  threads.reserve(kProducers);

  for (int p = 0; p < kProducers; ++p)
  {
    threads.emplace_back([&queue, p]() {
      for (int i = 0; i < kCommandsPerProducer; ++i)
      {
        Command cmd;
        cmd.kind = Command::Kind::Velocity;
        // Each producer encodes its identity in the robot name so we can
        // verify no command was corrupted (torn string write).
        cmd.robot = "robot_producer_" + std::to_string(p);
        cmd.twist = stdr_simulation::Twist2D{ .linear_x = static_cast<double>(p),
                                              .linear_y = static_cast<double>(i),
                                              .angular_z = 0.0 };
        queue.push(std::move(cmd));
      }
    });
  }

  for (std::thread& t : threads)
  {
    t.join();
  }

  // Drain once after all producers are done.
  const std::vector<Command> result = queue.drain();
  EXPECT_EQ(result.size(), static_cast<std::size_t>(kProducers * kCommandsPerProducer));

  // Verify no command has a corrupted robot name — each must start with "robot_producer_".
  for (const Command& cmd : result)
  {
    EXPECT_THAT(cmd.robot, ::testing::StartsWith("robot_producer_"));
  }
}

}  // namespace
}  // namespace stdr::plot_data

#pragma once

// Header-only: all method bodies are trivial (mutex + vector swap) and this
// file has no compiled dependencies, so a separate .cpp would add build
// overhead without benefit (see cpp-style.md for the general .hpp+.cpp rule).

/** @file MPSC command queue for the plot-to-sim data path.
 *
 *  Allows multiple producers (GUI thread, future plotter threads) to enqueue
 *  robot commands that the sim thread drains at the top of each step.
 *
 *  ## Implementation choice: mutex-guarded vector
 *
 *  Commands arrive at human-interaction rate (keyboard typing, plotter
 *  callbacks) — far below the sim tick rate (~10 Hz).  A lightweight mutex
 *  on the queue itself avoids contending on `sim_mutex_` (the sim thread's
 *  primary lock) while keeping the implementation trivially correct.  The
 *  mutex is held for the minimum time possible: push() appends and returns;
 *  drain() swaps the internal vector into a local and returns.  A true
 *  lock-free CAS queue is unnecessary at this rate and risks correctness
 *  bugs that are hard to detect in testing.
 *
 *  ## Conflict policy
 *
 *  Last-write-wins, per tick, per robot.  If two Velocity commands for the
 *  same robot are drained in the same sim step, the later command overwrites
 *  the earlier via the existing set_cmd_vel semantics (which replaces any
 *  outstanding cmd_vel).  This matches what teleop already does and is
 *  intentional — the most recent source wins.
 *
 *  @note `drain()` must be called from the sim thread only.  `push()` may
 *        be called from any thread concurrently. */

#include <stdr_simulation/types.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace stdr::plot_data
{

/** A robot command enqueued by a plotter or GUI widget and dispatched by the
 *  sim thread at the top of each step. */
struct Command
{
  /** Distinguishes how the sim thread should handle this command. */
  enum class Kind
  {
    Velocity,  ///< Apply twist to robot via set_cmd_vel.
    Teleport,  ///< Move robot to pose via set_robot_pose.
    Pause,     ///< Pause the simulation.
    Resume,    ///< Resume the simulation.
  };

  Kind kind;

  /** Target robot name.  Unused for Pause/Resume. */
  std::string robot;

  /** Velocity command.  Used when kind == Velocity.
   *
   *  Zero-initialized by Twist2D's default constructor, so unused fields are
   *  safe to ignore for non-Velocity commands. */
  stdr_simulation::Twist2D twist;

  /** Target pose.  Used when kind == Teleport.
   *
   *  Zero-initialized by Pose2D's default constructor, so unused fields are
   *  safe to ignore for non-Teleport commands. */
  stdr_simulation::Pose2D pose;
};

/** Thread-safe MPSC queue of `Command` values.
 *
 *  Multiple threads may call push() concurrently.  Exactly one thread (the
 *  sim thread) calls drain().  Both operations are O(n) in queue length but
 *  hold the internal lock for the minimum time necessary. */
class CommandQueue
{
public:
  CommandQueue() = default;

  // Non-copyable, non-movable — the mutex is not copyable.
  CommandQueue(const CommandQueue&) = delete;
  CommandQueue& operator=(const CommandQueue&) = delete;
  CommandQueue(CommandQueue&&) = delete;
  CommandQueue& operator=(CommandQueue&&) = delete;

  ~CommandQueue() = default;

  /** Enqueue a command.  Safe to call from any thread. */
  void push(Command cmd)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    pending_.push_back(std::move(cmd));
  }

  /** Drain all pending commands in FIFO order.
   *
   *  Safe to call from the sim thread only.  Holds the mutex only long
   *  enough to swap the internal buffer — it does not hold it during the
   *  caller's dispatch loop.
   *
   *  @return All commands enqueued since the last drain, oldest first.
   *          Returns an empty vector if no commands are pending. */
  [[nodiscard]] std::vector<Command> drain()
  {
    std::vector<Command> out;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      // Swap avoids copying the vector contents while holding the lock.
      out.swap(pending_);
    }
    return out;
  }

private:
  std::mutex mutex_;
  std::vector<Command> pending_;
};

}  // namespace stdr::plot_data

#pragma once

/** @file Core data types for the simulation library. */

namespace stdr_simulation {

/** 2D pose in world frame: position and orientation. */
struct Pose2D {
  double x{0.0};
  double y{0.0};
  double theta{0.0};
};

/** 2D velocity command: planar linear velocities and angular rate. */
struct Twist2D {
  double linear_x{0.0};
  double linear_y{0.0};
  double angular_z{0.0};
};

/** Gaussian noise configuration for sensor simulation. */
struct NoiseConfig {
  double mean{0.0};
  double std_dev{0.0};
};

}  // namespace stdr_simulation

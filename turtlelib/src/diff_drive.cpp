#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include "turtlelib/diff_drive.hpp"
#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <cstdlib> // contains std::abs
#include <iostream> 
#include <vector>

using namespace turtlelib;

DiffDrive::DiffDrive()
{
    wheel_track_ = 0.0;
    wheel_radius_ = 0.0;
    wheel_positions_.phi_left = 0.0;
    wheel_positions_.phi_right = 0.0;
}

DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
{
    wheel_track_ = wheel_track;
    wheel_radius_ = wheel_radius;
    wheel_positions_.phi_left = 0.0;
    wheel_positions_.phi_right = 0.0;
}

void DiffDrive::update_configuration(WheelPositions_phi new_wheel_positions)
{
    double delta_phi_left = new_wheel_positions.phi_left - wheel_positions_.phi_left;
    double delta_phi_right = new_wheel_positions.phi_right - wheel_positions_.phi_right;
    Configuration_q delta_q_b;
    Configuration_q delta_q;

    Twist2D twist;
    twist.omega = (wheel_radius_ / wheel_track_) * (-delta_phi_left + delta_phi_right);
    twist.x = (wheel_radius_ / 2.0) * (delta_phi_left + delta_phi_right);
    twist.y = 0.0;

    if (almost_equal(twist.omega, 0.0, 1.0e-12))
    {
        delta_q_b.theta_ = 0.0;
        delta_q_b.x_ = twist.x;
        delta_q_b.y_ = twist.y;
    }
    else
    {
        delta_q_b.theta_ = twist.omega;
        delta_q_b.x_ = ((twist.x * std::sin(twist.omega)) + (twist.y * (std::cos(twist.omega) - 1.0)))/ twist.omega;
        delta_q_b.y_ = ((twist.y * std::sin(twist.omega)) + (twist.x * (1.0 - std::cos(twist.omega))))/ twist.omega;
    }
    
    delta_q.theta_ = delta_q_b.theta_;
    delta_q.x_ = (std::cos(configuration_.theta_) * delta_q_b.x_) - (std::sin(configuration_.theta_) * delta_q_b.y_);
    delta_q.y_ = (std::sin(configuration_.theta_) * delta_q_b.x_) + (std::cos(configuration_.theta_) * delta_q_b.y_);

    configuration_.theta_ += delta_q.theta_;
    configuration_.x_ += delta_q.x_;
    configuration_.y_ += delta_q.y_;
    wheel_positions_ = new_wheel_positions;
}

WheelPositions_phi DiffDrive::compute_wheel_velocities(Twist2D twist)
{
    WheelPositions_phi wheel_velocities;

    if (almost_equal(twist.y, 0.0, 1.0e-12))
    {
        wheel_velocities.phi_left = (twist.x - (wheel_track_ * twist.omega / 2.0)) / wheel_radius_;
        wheel_velocities.phi_right = (twist.x + (wheel_track_ * twist.omega / 2.0)) / wheel_radius_;
    }
    else
    {
        throw std::logic_error ("twist cannot be accomplished");
    }
    return wheel_velocities;
}

void DiffDrive::set_configuration(Configuration_q configuration)
{
    configuration_ = configuration;
}

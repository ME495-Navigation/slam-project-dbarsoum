#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

using namespace turtlelib;

TEST_CASE("Test DiffDrive", "[DiffDrive]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);

    WheelPositions_phi new_wheel_positions;
    new_wheel_positions.phi_left = 0.0;
    new_wheel_positions.phi_right = 0.0;
    diff_drive.update_configuration(new_wheel_positions);

    Configuration_q config;
    config.theta_ = 0.0;
    config.x_ = 0.0;
    config.y_ = 0.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_forward]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = PI / 2.0;
    new_wheel_positions.phi_right = PI / 2.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_= 0.0;
    config.x_ = wheel_radius * PI / 2.0;
    config.y_ = 0.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_backward]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = -PI / 2.0;
    new_wheel_positions.phi_right = -PI / 2.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_= 0.0;
    config.x_ = -wheel_radius * PI / 2.0;
    config.y_ = 0.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_rightturn]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = PI / 2.0;
    new_wheel_positions.phi_right = -PI / 2.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_ = -2.0 * PI;
    config.x_ = 0.0;
    config.y_ = 0.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1.0e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1.0e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_leftturn]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = -PI / 2.0;
    new_wheel_positions.phi_right = PI / 2.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_= 2.0 * PI;
    config.x_ = 0.0;
    config.y_ = 0.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = config.theta_;
    twist.x = config.x_;
    twist.y = config.y_;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_forward_leftturn]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = PI / 2.0;
    new_wheel_positions.phi_right = PI / 4.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_= -PI / 2.0;
    config.x_ = 3.0 / 2.0;
    config.y_ = -3.0 / 2.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = -PI / 2.0;
    twist.x = (3.0 * PI) / 4.0;
    twist.y = 0.0;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

TEST_CASE("Test DiffDrive", "[DiffDrive_backward_leftturn]")
{
    double wheel_track = 1.0;
    double wheel_radius = 2.0;
    DiffDrive diff_drive(wheel_track, wheel_radius);
    WheelPositions_phi new_wheel_positions;
    Configuration_q config;

    new_wheel_positions.phi_left = -PI / 2.0;
    new_wheel_positions.phi_right = -PI / 4.0;
    diff_drive.update_configuration(new_wheel_positions);

    config.theta_= PI / 2.0;
    config.x_ = -3.0 / 2.0;
    config.y_ = -3.0 / 2.0;

    REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
    REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

    diff_drive = DiffDrive(wheel_track, wheel_radius);
    Twist2D twist;
    twist.omega = PI / 2.0;
    twist.x = -(3.0 * PI) / 4.0;
    twist.y = 0.0;

    WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
    REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(new_wheel_positions.phi_left, 1e-12));
    REQUIRE_THAT(wheel_velocities.phi_right, Catch::Matchers::WithinAbs(new_wheel_positions.phi_right, 1e-12));
}

// TEST_CASE("Test DiffDrive", "[DiffDrive_no_twist]")
// {
//     double wheel_track = 1.0;
//     double wheel_radius = 2.0;
//     DiffDrive diff_drive(wheel_track, wheel_radius);
//     WheelPositions_phi new_wheel_positions;
//     Configuration_q config;

//     new_wheel_positions.phi_left = 0.0;
//     new_wheel_positions.phi_right = 0.0;
//     diff_drive.update_configuration(new_wheel_positions);

//     config.theta_= 0.0;
//     config.x_ = 0.0;
//     config.y_ = 0.0;

//     REQUIRE_THAT(diff_drive.get_configuration().theta_, Catch::Matchers::WithinAbs(config.theta_, 1e-12));
//     REQUIRE_THAT(diff_drive.get_configuration().x_, Catch::Matchers::WithinAbs(config.x_, 1e-12));
//     REQUIRE_THAT(diff_drive.get_configuration().y_, Catch::Matchers::WithinAbs(config.y_, 1e-12));

//     diff_drive = DiffDrive(wheel_track, wheel_radius);
//     Twist2D twist;
//     twist.omega = 0.0;
//     twist.x = 0.0;
//     twist.y = 2.0;

//     WheelPositions_phi wheel_velocities = diff_drive.compute_wheel_velocities(twist);
//     /// make sure twist cannot be accomplished with current configuration
//     REQUIRE_THAT(wheel_velocities.phi_left, Catch::Matchers::WithinAbs(0.0, 1e-12));
// }
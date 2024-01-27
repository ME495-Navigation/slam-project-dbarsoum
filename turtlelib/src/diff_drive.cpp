// #include "turtlelib/geometry2d.hpp"
// #include "turtlelib/se2d.hpp"
// #include "turtlelib/diff_drive.hpp"
// #include <vector>

// namespace turtlelib
// {
//     /// \brief models the kinematics of a differential drive robot
//     class DiffDrive
//     {
//         public:
//             DiffDrive();
//             DiffDrive(double wheel_base, double wheel_radius);
//             void set_wheel_base(double wheel_base);
//             void set_wheel_radius(double wheel_radius);
//             void set_pose(Pose2D pose);
//             void set_pose(double x, double y, double theta);
//             void set_twist(Twist2D twist);
//             void set_twist(double omega, double x, double y);
//             void set_wheel_speeds(double left, double right);
//             void set_wheel_speeds(double linear, double angular);
//             void set_wheel_speeds(Twist2D twist);
//             void set_wheel_speeds(double linear, double angular, double wheel_base);
//             void set_wheel_speeds(Twist2D twist, double wheel_base);
//             void update_pose(double dt);
//             void update_pose(double dt, double wheel_base);
//             void update_pose(double dt, double wheel_base, double wheel_radius);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double left, double right);
//             void update_pose(double dt, double wheel_base, double wheel_radius, Twist2D twist);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, Twist2D twist);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right, Twist2D twist);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right, Twist2D twist, Pose2D pose);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right, Twist2D twist, double x, double y, double theta);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right, Twist2D twist, Pose2D pose, double x, double y, double theta);
//             void update_pose(double dt, double wheel_base, double wheel_radius, double linear, double angular, double left, double right, Twist2D twist, Pose2D pose, double x, double y,)
//     };
// }

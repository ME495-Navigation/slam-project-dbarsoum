#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <cstdlib> // contains std::abs
#include <iostream> 
#include <vector>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief models the wheel positions of a differential drive robot
    struct WheelPositions_phi
    {
        /// \brief left wheel position
        double phi_left;

        /// \brief right wheel position
        double phi_right;
    };

    /// \brief models the configuration of a differential drive robot
    /// \details the configuration is given by the position and orientation of the robot
    struct Configuration_q
    {
        /// \brief x position of robot
        double x_;

        /// \brief y position of robot
        double y_;

        /// \brief orientation of robot
        double theta_;
    };

    /// \brief models the kinematics of a differential drive robot
    class DiffDrive
    {
    public:
        /// \brief create empty DiffDrive object
        DiffDrive();

        /// \brief create DiffDrive object with given wheel track and wheel radius
        /// \param wheel_track 
        /// \param wheel_radius 
        DiffDrive(double wheel_track, double wheel_radius);

        /// \brief given wheel positions, update the configuration of the robot
        /// \param new_wheel_positions
        void update_configuration(WheelPositions_phi new_wheel_positions);

        /// \brief compute the wheel velocities required to achieve the given twist
        /// \param twist
        /// \return wheel velocities
        WheelPositions_phi compute_wheel_velocities(Twist2D twist);

        /// \brief get configuration of robot   
        /// \return configuration
        Configuration_q get_configuration() const 
        { 
            return configuration_;
        }

        /// \brief 
        void set_configuration(Configuration_q configuration);

    private:
        double wheel_track_;
        double wheel_radius_;

        WheelPositions_phi wheel_positions_;
        Configuration_q configuration_;
    };
}

#endif // TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
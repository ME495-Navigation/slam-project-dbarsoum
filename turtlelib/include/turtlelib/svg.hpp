#ifndef TURTLELIB_CREATE_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_CREATE_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath> // contains math functions
#include <cstdlib> // contains std::abs
#include <iostream> 
#include <vector>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    class CREATE_SVG
    {
    public:
        /// \brief 
        CREATE_SVG();

        /// \brief return the svg string
        std::string EXPORT();

        /// \brief takes in filename and returns a void
        void EXPORT(std::string filename);

        /// \brief Create a point on the svg
        void DRAW(turtlelib::Point2D pt, std::string color);
        // void DRAW_LINE(Point2D pt1, Point2D pt2, std::string color = "black", double width = 1.0);

        /// \brief Create a line on the svg
        void DRAW(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color);

        /// \brief Create a coordinate frame
        void DRAW(turtlelib::Transform2D tf);
        
        private:
            std::string svg_str;
    };

}

#endif // TURTLELIB_CREATE_SVG_INCLUDE_GUARD_HPP
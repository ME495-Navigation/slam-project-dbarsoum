#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>


/// \brief print the Twist2D in the format [w x y]
/// \param os [in/out] the ostream to write to
/// \param tw the twist to output
/// \returns the ostream os  with the twist data inserted
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Twist2D & tw)
{
    os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    return os;
}

/// \brief print the Twist2D in the format [w x y]
/// \param os [in/out] the ostream to write to
/// \param tw the twist to output
/// \returns the ostream os  with the twist data inserted
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Twist2D & tw)
{
    char c;
    is >> c;
    if (c == '[')
    {
        is >> tw.omega >> tw.x >> tw.y >> c;
    }
    else
    {
        is.putback(c);
        is >> tw.omega >> tw.x >> tw.y;
    }
    return is;
}

/// \brief Create an identity transformation
turtlelib::Transform2D::Transform2D()
{
    matrix = std::vector<std::vector<double>>(3, std::vector<double>(3, 0));

    matrix[0][0] = 1;   
    matrix[1][1] = 1;
    matrix[2][2] = 1;
}

#include "turtlelib/geometry2d.hpp"
// #include <spdlog/fmt/bundled/ostream.h>

/// \brief wrap an angle to (-PI, PI]
/// \param rad (angle in radians)
/// \return an angle equivalent to rad but in the range (-PI, PI]
double turtlelib::normalize_angle(double rad)
{
    while (rad > turtlelib::PI)
    {
        rad -= 2.0 * turtlelib::PI;
    }
    while (rad <= -turtlelib::PI)
    {
        rad += 2.0 * turtlelib::PI;
    }
    return rad;
}

/// \brief output a 2 dimensional point as [xcomponent ycomponent]
/// \param os - stream to output to
/// \param p - the point to print
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Point2D & p)
{
    os << "[" << p.x << " " << p.y << "]";
    return os;
}

/// \brief input a 2 dimensional point
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param p [out] - output vector
/// HINT: See operator>> for Vector2D
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Point2D & p)
{
    char c;
    is >> c;
    if (c == '[')
    {
        is >> p.x >> p.y >> c;
    }
    else
    {
        is.putback(c);
        is >> p.x >> p.y;
    }
    return is;
}

/// \brief Subtracting one point from another yields a vector
/// \param head point corresponding to the head of the vector
/// \param tail point corresponding to the tail of the vector
/// \return a vector that points from p1 to p2
/// NOTE: this is not implemented in terms of -= because subtracting two Point2D yields a Vector2D
turtlelib::Vector2D turtlelib::operator-(const turtlelib::Point2D & head, const turtlelib::Point2D & tail)
{
    Vector2D v;
    v.x = head.x - tail.x;
    v.y = head.y - tail.y;
    return v;
}

/// \brief Adding a vector to a point yields a new point displaced by the vector
/// \param tail The origin of the vector's tail
/// \param disp The displacement vector
/// \return the point reached by displacing by disp from tail
/// NOTE: this is not implemented in terms of += because of the different types
turtlelib::Point2D turtlelib::operator+(const turtlelib::Point2D & tail, const turtlelib::Vector2D & disp)
{
    Point2D p;
    p.x = tail.x + disp.x;
    p.y = tail.y + disp.y;
    return p;
}

/// \brief output a 2 dimensional vector as [xcomponent ycomponent]
/// \param os - stream to output to
/// \param v - the vector to print
std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v)
{
    os << "[" << v.x << " " << v.y << "]";
    return os;
}

/// \brief input a 2 dimensional vector
///   You should be able to read vectors entered as follows:
///   [x y] or x y
/// \param is - stream from which to read
/// \param v [out] - output vector
std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Vector2D & v)
{
    char c;
    is >> c;
    if (c == '[')
    {
        is >> v.x >> v.y >> c;
    }
    else
    {
        is.putback(c);
        is >> v.x >> v.y;
    }
    return is;
}
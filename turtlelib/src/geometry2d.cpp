#include "turtlelib/geometry2d.hpp"
// #include <spdlog/fmt/bundled/ostream.h>

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

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Point2D & p)
{
    os << "[" << p.x << " " << p.y << "]";
    return os;
}

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

turtlelib::Vector2D turtlelib::operator-(const turtlelib::Point2D & head, const turtlelib::Point2D & tail)
{
    Vector2D v;
    v.x = head.x - tail.x;
    v.y = head.y - tail.y;
    return v;
}

turtlelib::Point2D turtlelib::operator+(const turtlelib::Point2D & tail, const turtlelib::Vector2D & disp)
{
    Point2D p;
    p.x = tail.x + disp.x;
    p.y = tail.y + disp.y;
    return p;
}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Vector2D & v)
{
    os << "[" << v.x << " " << v.y << "]";
    return os;
}


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

turtlelib::Vector2D turtlelib::operator+=(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    v1.x += v2.x;
    v1.y += v2.y;
    return v1;
}

turtlelib::Vector2D turtlelib::operator+(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    v1 += v2;
    return v1;
}

turtlelib::Vector2D turtlelib::operator-=(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    v1.x -= v2.x;
    v1.y -= v2.y;
    return v1;
}

turtlelib::Vector2D operator-(turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    v1 -= v2;
    return v1;
}

turtlelib::Vector2D turtlelib::operator*=(turtlelib::Vector2D & v, double s)
{
    v.x *= s;
    v.y *= s;
    return v;
}

turtlelib::Vector2D turtlelib::operator*(turtlelib::Vector2D & v, double s)
{
    v *= s;
    return v;
}

turtlelib::Vector2D turtlelib::operator*(double s, turtlelib::Vector2D & v)
{
    v *= s;
    return v;
}

double turtlelib::dot(const turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

double turtlelib::magnitude(const turtlelib::Vector2D & v)
{
    return sqrt(v.x * v.x + v.y * v.y);
}

double turtlelib::angle(const turtlelib::Vector2D & v1, const turtlelib::Vector2D & v2)
{
    return acos(turtlelib::dot(v1, v2) / (turtlelib::magnitude(v1) * turtlelib::magnitude(v2)));
}

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include <vector>


std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Twist2D & tw)
{
    os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
    return os;
}

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

turtlelib::Transform2D::Transform2D()
{
    theta = 0;
    pt.x = 0;
    pt.y = 0;
}

turtlelib::Transform2D::Transform2D(Vector2D trans)
{
    theta = 0;
    pt.x = trans.x;
    pt.x = trans.y;
}

turtlelib::Transform2D::Transform2D(double radians)
{
    theta = radians;
    pt.x = 0;
    pt.y = 0;
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
{
    theta = radians;
    pt.x = trans.x;
    pt.y = trans.y;
}


turtlelib::Point2D turtlelib::Transform2D::operator()(turtlelib::Point2D p) const
{
    turtlelib::Point2D newPoint;
    newPoint.x = cos(theta) * p.x - sin(theta) * p.y + pt.x;
    newPoint.y = sin(theta) * p.x + cos(theta) * p.y + pt.y;
    return newPoint;
}


turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newVector;
    newVector.x = cos(theta) * v.x - sin(theta) * v.y;
    newVector.y = sin(theta) * v.x + cos(theta) * v.y;
    return newVector;
}

turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D v) const
{
    turtlelib::Twist2D newTwist;
    newTwist.omega = v.omega;
    newTwist.x = pt.y* v.omega + cos(theta) * v.x - sin(theta) * v.y;
    newTwist.y = -pt.x * v.omega + sin(theta) * v.x + cos(theta) * v.y;
    return newTwist;
}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Transform2D Tinv;
    Tinv.theta = -theta;
    Tinv.pt.x = -cos(theta) * pt.x - sin(theta) * pt.y;
    Tinv.pt.y = sin(theta) * pt.x - cos(theta) * pt.y;
    return Tinv;
    
}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D & rhs)
{
    pt.x = cos(theta) * rhs.pt.x - sin(theta) * rhs.pt.y + pt.x ;
    pt.y = sin(theta) * rhs.pt.x + cos(theta) * rhs.pt.y + pt.y;
    // pt.x = cos(theta) * rhs.pt.x - sin(theta) * rhs.pt.y;
    // pt.y = sin(theta) * rhs.pt.x + cos(theta) * rhs.pt.y;
    theta = theta + rhs.theta;
    return *this;
}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    turtlelib::Vector2D v;
    v.x = pt.x;
    v.y = pt.y;
    return v;
}

double turtlelib::Transform2D::rotation() const
{
    return theta;
}

std::ostream & turtlelib::operator<<(std::ostream & os, const Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, Transform2D & tf)
{
    turtlelib::Vector2D v;
    double omega;
    is >> omega >> v.x >> v.y;
    omega = turtlelib::deg2rad(omega);
    tf = turtlelib::Transform2D(v, omega);
    return is;
}

turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}
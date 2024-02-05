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
    char c; // nunitialized
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

// the canonical way of implementing class methods within a namespace is as follows
//namespace turtlelib
//{
// everything in here is now turtlelib
//}
/// \brief Create an identity transformation
turtlelib::Transform2D::Transform2D() // use initializer list
{
    theta = 0;
    pt.x = 0;
    pt.y = 0;
}

/// \brief create a transformation that is a pure translation
turtlelib::Transform2D::Transform2D(Vector2D trans) // use initializer lists
{
    theta = 0;
    pt.x = trans.x;
    pt.x = trans.y;
}

/// \brief create a pure rotation
turtlelib::Transform2D::Transform2D(double radians) // initializer lists
{
    theta = radians;
    pt.x = 0;
    pt.y = 0;
}

/// \brief Create a transformation with a translational and rotational cmpnt
turtlelib::Transform2D::Transform2D(Vector2D trans, double radians) // initializer lists
{
    theta = radians;
    pt.x = trans.x;
    pt.y = trans.y;
}

/// \brief apply a transformation to a 2D Point
turtlelib::Point2D turtlelib::Transform2D::operator()(turtlelib::Point2D p) const
{
    turtlelib::Point2D newPoint;
    newPoint.x = cos(theta) * p.x - sin(theta) * p.y + pt.x;
    newPoint.y = sin(theta) * p.x + cos(theta) * p.y + pt.y;
    return newPoint; // can just return {cos(theta)*p.x - , ..rest of math}
}

/// \brief apply a transformation to a 2D Vector
turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newVector;
    newVector.x = cos(theta) * v.x - sin(theta) * v.y;
    newVector.y = sin(theta) * v.x + cos(theta) * v.y;
    return newVector; // no need for temporary, just return {stuff, stuff1}
}

/// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D v) const
{
    turtlelib::Twist2D newTwist;
    newTwist.omega = v.omega;
    newTwist.x = pt.y* v.omega + cos(theta) * v.x - sin(theta) * v.y;
    newTwist.y = -pt.x * v.omega + sin(theta) * v.x + cos(theta) * v.y;
    return newTwist; // no need for temporary...
}

/// \brief invert the transformation
/// \return the inverse transformation.
turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Transform2D Tinv;
    Tinv.theta = -theta;
    Tinv.pt.x = -cos(theta) * pt.x - sin(theta) * pt.y;
    Tinv.pt.y = sin(theta) * pt.x - cos(theta) * pt.y;
    return Tinv; // no need for temporary can return {}
    
}

/// \brief compose this transform with another and store the result
/// in this object
/// \param rhs - the first transform to apply
/// \return a reference to the newly transformed operator
turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const turtlelib::Transform2D & rhs)
{
    pt.x = cos(theta) * rhs.pt.x - sin(theta) * rhs.pt.y + pt.x ;
    pt.y = sin(theta) * rhs.pt.x + cos(theta) * rhs.pt.y + pt.y;
    // do not leave commented out code
    // pt.x = cos(theta) * rhs.pt.x - sin(theta) * rhs.pt.y;
    // pt.y = sin(theta) * rhs.pt.x + cos(theta) * rhs.pt.y;
    theta = theta + rhs.theta; // +=
    return *this;
}

/// \brief the translational component of the transform
/// \return the x,y translation
turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    turtlelib::Vector2D v;
    v.x = pt.x;
    v.y = pt.y;
    return v; // return {pt.x, pt.y}
}

/// \brief get the angular displacement of the transform
/// \return the angular displacement, in radians
double turtlelib::Transform2D::rotation() const
{
    return theta;
}

/// \brief should print a human readable version of the transform:
/// An example output:
/// deg: 90 x: 3 y: 5
/// \param os - an output stream
/// \param tf - the transform to print
std::ostream & turtlelib::operator<<(std::ostream & os, const Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
    return os;
}

/// \brief Read a transformation from stdin
/// Should be able to read input either as output by operator<< or
/// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
std::istream & turtlelib::operator>>(std::istream & is, Transform2D & tf)
{
    turtlelib::Vector2D v;
    double omega;
    is >> omega >> v.x >> v.y;
    omega = turtlelib::deg2rad(omega);
    tf = turtlelib::Transform2D(v, omega);
    return is;
}

/// \brief multiply two transforms together, returning their composition
/// \param lhs - the left hand operand
/// \param rhs - the right hand operand
/// \return the composition of the two transforms
/// HINT: This function should be implemented in terms of *=
turtlelib::Transform2D turtlelib::operator*(turtlelib::Transform2D lhs, const turtlelib::Transform2D & rhs)
{
    lhs *= rhs;
    return lhs;
}

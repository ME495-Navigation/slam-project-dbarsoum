#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>

TEST_CASE("Test operator<<", "[se2d_operator<<]")
{
    turtlelib::Twist2D tw;
    tw.omega = 1.0;
    tw.x = 2.0;
    tw.y = 3.0;
    std::stringstream ss;
    ss << tw;
    REQUIRE(ss.str() == "[1 2 3]");
}

TEST_CASE("Test operator>>", "[se2d_operator>>]")
{
    turtlelib::Twist2D tw;
    std::stringstream ss;
    ss << "[1 2 3]";
    ss >> tw;
    REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(3.0, 1e-6));

    std::stringstream ss2;
    ss2 << "1 2 3";
    ss2 >> tw;
    REQUIRE_THAT(tw.omega, Catch::Matchers::WithinAbs(1.0, 1e-6));
    REQUIRE_THAT(tw.x, Catch::Matchers::WithinAbs(2.0, 1e-6));
    REQUIRE_THAT(tw.y, Catch::Matchers::WithinAbs(3.0, 1e-6));
}


TEST_CASE("operator()", "[operator_pt]")
{
    double rot = turtlelib::PI/2.0;
    turtlelib::Vector2D vec;
    vec.x = 0.0;
    vec.y = 1.0;
    turtlelib::Transform2D T_ab;
    T_ab = turtlelib::Transform2D(vec, rot);
    turtlelib::Point2D pt;
    pt.x = 1.0;
    pt.y = 1.0;
    turtlelib::Point2D pt2 = T_ab(pt);
    REQUIRE_THAT(pt2.x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(pt2.y, Catch::Matchers::WithinAbs(2.0, 1e-12));
}

TEST_CASE("operator()", "[operator_vec]")
{
    double rot = turtlelib::PI/2.0;
    turtlelib::Vector2D vec;
    vec.x = 0.0;
    vec.y = 1.0;
    turtlelib::Transform2D T_ab;
    T_ab = turtlelib::Transform2D(vec, rot);
    turtlelib::Vector2D vec2;
    vec2.x = 1.0;
    vec2.y = 1.0;
    turtlelib::Vector2D vec3 = T_ab(vec2);
    REQUIRE_THAT(vec3.x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(vec3.y, Catch::Matchers::WithinAbs(2.0, 1e-12));
}

TEST_CASE("operator()", "[operator_twist]")
{
    double rot = turtlelib::PI/2.0;
    turtlelib::Vector2D vec;
    vec.x = 0.0;
    vec.y = 1.0;
    turtlelib::Transform2D T_ab;
    T_ab = turtlelib::Transform2D(vec, rot);
    turtlelib::Twist2D tw;
    tw.omega = 1.0;
    tw.x = 1.0;
    tw.y = 1.0;
    turtlelib::Twist2D tw2 = T_ab(tw);
    REQUIRE_THAT(tw2.omega, Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(tw2.x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(tw2.y, Catch::Matchers::WithinAbs(0.0, 1e-12));
}


TEST_CASE("inv", "[inv]")
{
    double test_rot = turtlelib::PI/2.0;
    turtlelib::Vector2D test_vec = {0.0, 1.0};
    turtlelib::Transform2D T_ab{{test_vec}, test_rot};
    turtlelib::Transform2D T_ba = T_ab.inv();
    REQUIRE_THAT(T_ba.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(T_ba.translation().y, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(T_ba.rotation(), Catch::Matchers::WithinAbs(-test_rot, 1e-12));
}

// TEST_CASE("operator<<", )

TEST_CASE("operator*=", "[operator*=]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T1 = turtlelib::Transform2D(v2, theta2);

    T*=T1;
    double gamma = turtlelib::rad2deg(T.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    /// if vector is represented as a column vector = [x y 1]
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));

    /// if vector is represented as a column vector = [x y 0]
    // REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-4.0, 1.0e-12));
    // REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(3.0, 1.0e-12));
   
}

// std::ostream & turtlelib::operator<<(std::ostream & os, const Transform2D & tf)
TEST_CASE("operator<<", "[operator<<]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);
    std::stringstream ss;
    ss << T;
    REQUIRE(ss.str() == "deg: 90 x: 1 y: 2");
}

// std::istream & operator>>(std::istream & is, Transform2D & tf);
TEST_CASE("operator>>", "[operator>>]")
{
    turtlelib::Transform2D T;
    std::stringstream ss;
    ss << "90 1 2";
    ss >> T;

    auto trans = T.translation();
    auto rot = T.rotation();
    REQUIRE_THAT(trans.x, Catch::Matchers::WithinAbs(1.0, 1.0e-12));
    REQUIRE_THAT(trans.y, Catch::Matchers::WithinAbs(2.0, 1.0e-12));
    REQUIRE_THAT(rot, Catch::Matchers::WithinAbs(turtlelib::deg2rad(90.0), 1.0e-12));
}

// Transform2D operator*(Transform2D lhs, const Transform2D & rhs);
TEST_CASE("operator*", "[operator*]")
{
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 2.0;
    double theta = turtlelib::deg2rad(90.0);

    turtlelib::Transform2D T = turtlelib::Transform2D(v, theta);

    turtlelib::Vector2D v2;
    v2.x = 3.0;
    v2.y = 4.0;
    double theta2 = turtlelib::deg2rad(180.0);
    turtlelib::Transform2D T1 = turtlelib::Transform2D(v2, theta2);

    turtlelib::Transform2D T2 = T * T1;
    double gamma = turtlelib::rad2deg(T2.rotation());
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(270.0, 1.0e-12));
    REQUIRE_THAT(T2.translation().x, Catch::Matchers::WithinAbs(-3.0, 1.0e-12));
    REQUIRE_THAT(T2.translation().y, Catch::Matchers::WithinAbs(5.0, 1.0e-12));
}

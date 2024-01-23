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
    double test_rot1 = turtlelib::PI/2.0;
    turtlelib::Vector2D test_vec = {0.0, 1.0};
    turtlelib::Transform2D T_ab{{test_vec}, test_rot1};
    turtlelib::Point2D T_b{1, 1};
    turtlelib::Point2D T_a = T_ab(T_b);
    REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("operator()", "[operator_vec]")
{
    double test_rot = turtlelib::PI/2.0;
    turtlelib::Vector2D test_vec = {0.0, 1.0};
    turtlelib::Transform2D T_ab{{test_vec}, test_rot};
    turtlelib::Vector2D T_b{1, 1};
    turtlelib::Vector2D T_a = T_ab(T_b);
    REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-12));
}

TEST_CASE("operator()", "[operator_twist]")
{

    double test_rot = turtlelib::PI/2.0;
    turtlelib::Vector2D test_vec = {0.0, 1.0};
    turtlelib::Transform2D T_ab{{test_vec}, test_rot};
    turtlelib::Twist2D T_b{1, 1, 1};
    turtlelib::Twist2D T_a = T_ab(T_b);
    REQUIRE_THAT(T_a.omega, Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(T_a.x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(T_a.y, Catch::Matchers::WithinAbs(1.0, 1e-12));
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
    // double theta = turtlelib::PI/2.0;
    // turtlelib::Vector2D vec = {0.0, 1.0};
    // turtlelib::Transform2D T1{{vec}, theta};

    // double theta2 = 0.0;
    // turtlelib::Vector2D vec2 = {0.0, 1.0};
    // turtlelib::Transform2D T2{{vec2}, theta2};

    // turtlelib::Transform2D T;

    // REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-1.0, 1e-12));
    // REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(1.0, 1e-12));
    // REQUIRE_THAT(T.rotation(), Catch::Matchers::WithinAbs(theta, 1e-12));

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
    REQUIRE_THAT(gamma, Catch::Matchers::WithinAbs(90.0, 1.0e-12));
    REQUIRE_THAT(T.translation().x, Catch::Matchers::WithinAbs(-2.0, 1.0e-12));
    REQUIRE_THAT(T.translation().y, Catch::Matchers::WithinAbs(6.0, 1.0e-12));
   
}

#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

TEST_CASE("Test almost_equal", "[almost_equal]")
{
    REQUIRE(turtlelib::almost_equal(1.0, 1.0) == true);
    REQUIRE(turtlelib::almost_equal(25.0, 28.0) == false);
    REQUIRE(turtlelib::almost_equal(0.0, 0.000000000000000000001) == true);
}

TEST_CASE("Test deg2rad", "[deg2rad]")
{
    REQUIRE_THAT(turtlelib::deg2rad(0.0), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(turtlelib::deg2rad(180.0), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-12));
    REQUIRE_THAT(turtlelib::deg2rad(360.0), Catch::Matchers::WithinAbs(2.0 * turtlelib::PI, 1e-12));

}

TEST_CASE("Test rad2deg", "[rad2deg]")
{
    REQUIRE_THAT(turtlelib::rad2deg(0.0), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(turtlelib::rad2deg(turtlelib::PI), Catch::Matchers::WithinAbs(180.0, 1e-12));
    REQUIRE_THAT(turtlelib::rad2deg(2.0 * turtlelib::PI), Catch::Matchers::WithinAbs(360.0, 1e-12));

}

TEST_CASE("Test normalize_angle", "[normalize_angle]")
{
    REQUIRE_THAT(turtlelib::normalize_angle(0.0), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-turtlelib::PI / 4), Catch::Matchers::WithinAbs(-turtlelib::PI/4, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(3.0 * turtlelib::PI / 2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1e-12));
    REQUIRE_THAT(turtlelib::normalize_angle(-5.0 * turtlelib::PI / 2), Catch::Matchers::WithinAbs(-turtlelib::PI / 2, 1e-12));
}

TEST_CASE("Test operator<<", "[operator<<]")
{
    turtlelib::Point2D p;
    p.x = 0.0;
    p.y = 0.0;
    std::stringstream ss;
    ss << p;
    /// [x y]
    REQUIRE(ss.str() == "[0 0]");

    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 1.0;
    std::stringstream ss2;
    ss2 << v;
    /// [xcomponent ycomponent]
    REQUIRE(ss2.str() == "[1 1]");
}

TEST_CASE("Test operator>>", "[operator>>]")
{
    turtlelib::Point2D p;
    std::stringstream ss;
    ss << "[0 0]";
    ss >> p;
    REQUIRE(p.x == 0.0);
    REQUIRE(p.y == 0.0);
}

TEST_CASE("Test operator-", "[operator-]")
{
    turtlelib::Point2D p1;
    p1.x = 1.0;
    p1.y = 1.0;
    turtlelib::Point2D p2;
    p2.x = 0.0;
    p2.y = 0.0;
    turtlelib::Vector2D v = p1 - p2;
    REQUIRE(v.x == 1.0);
    REQUIRE(v.y == 1.0);
}

TEST_CASE("Test operator+", "[operator+]")
{
    turtlelib::Point2D p1;
    p1.x = 1.0;
    p1.y = 1.0;
    turtlelib::Vector2D v;
    v.x = 1.0;
    v.y = 1.0;
    turtlelib::Point2D p2 = p1 + v;
    REQUIRE(p2.x == 2.0);
    REQUIRE(p2.y == 2.0);
}



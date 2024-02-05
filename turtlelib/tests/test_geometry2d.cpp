#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>

TEST_CASE("Test almost_equal", "[almost_equal]")
{
    // these tests are better as static assertions because this is constexpr
    REQUIRE(turtlelib::almost_equal(1.0, 1.0) == true);
    REQUIRE(turtlelib::almost_equal(25.0, 28.0) == false);
    REQUIRE(turtlelib::almost_equal(0.0, 0.000000000000000000001) == true);
}

TEST_CASE("Test deg2rad", "[deg2rad]")
{
    // these tests are better as static assertions because this is constexpr
    REQUIRE_THAT(turtlelib::deg2rad(0.0), Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(turtlelib::deg2rad(180.0), Catch::Matchers::WithinAbs(turtlelib::PI, 1e-12));
    REQUIRE_THAT(turtlelib::deg2rad(360.0), Catch::Matchers::WithinAbs(2.0 * turtlelib::PI, 1e-12));

}

TEST_CASE("Test rad2deg", "[rad2deg]")
{
    // these tests are better as static assertions because this is constexpr
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
    turtlelib::Point2D p; // use constructor to initialize p: turtlelib::Point2D p{0.0}
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
    REQUIRE_THAT(p.x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(p.y, Catch::Matchers::WithinAbs(0.0, 1e-12));

    turtlelib::Point2D p1;
    std::stringstream ss2;
    ss2 << "0 0";
    ss2 >> p1;
    REQUIRE_THAT(p1.x, Catch::Matchers::WithinAbs(0.0, 1e-12));
    REQUIRE_THAT(p1.y, Catch::Matchers::WithinAbs(0.0, 1e-12));

    turtlelib::Point2D p2;
    std::stringstream ss3;
    ss3 << "[2.0 2.0]";
    ss3 >> p2;
    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(2.0, 1e-12));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(2.0, 1e-12));

    turtlelib::Point2D p3;
    std::stringstream ss4;
    ss4 << "2.0 2.0";
    ss4 >> p3;
    REQUIRE_THAT(p3.x, Catch::Matchers::WithinAbs(2.0, 1e-12));
    REQUIRE_THAT(p3.y, Catch::Matchers::WithinAbs(2.0, 1e-12));
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
    REQUIRE_THAT(v.x, Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(v.y, Catch::Matchers::WithinAbs(1.0, 1e-12));
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
    REQUIRE_THAT(p2.x, Catch::Matchers::WithinAbs(2.0, 1e-12));
    REQUIRE_THAT(p2.y, Catch::Matchers::WithinAbs(2.0, 1e-12));
}



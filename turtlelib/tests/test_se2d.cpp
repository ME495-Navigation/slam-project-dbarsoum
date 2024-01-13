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

TEST_CASE("Test Transform2D()", "[Transform2D]")
{
    turtlelib::Transform2D tm;
    REQUIRE_THAT(tm.matrix[0][0], Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(tm.matrix[1][1], Catch::Matchers::WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(tm.matrix[2][2], Catch::Matchers::WithinAbs(1.0, 1e-12));

}

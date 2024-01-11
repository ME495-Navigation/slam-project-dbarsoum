#include "turtlelib/geometry2d.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("Test almost_equal", "[almost_equal]")
{
    REQUIRE(turtlelib::almost_equal(0.0, 0.0) == true);
}
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <sstream>
#include <vector>


TEST_CASE("Test EXPORT", "[EXPORT]")
{
    turtlelib::CREATE_SVG svg;
    std::string svg_str;
    turtlelib::Point2D pt;
    turtlelib::Point2D head;
    turtlelib::Point2D tail;
    turtlelib::Transform2D tf;
    std::string color = "purple";
    pt.x = -1.0;
    pt.y = -1.0;
    head.x = -2.0;
    head.y = 3.0;
    tail.x = 2.0;
    tail.y = 3.0;
    // turtlelib::Transform2D tf_trans = turtlelib::Transform2D(turtlelib::Point2D(1.0, 1.0));
    // tf_origin = turtlelib::Transform2D(turtlelib::Point2D(0.0, 0.0));
    // tf_x = turtlelib::Transform2D(turtlelib::Point2D(1.0, 0.0));
    // tf_y = turtlelib::Transform2D(turtlelib::Point2D(0.0, 1.0));
    svg.DRAW(pt, color);
    svg.DRAW(head, tail, color);
    svg.DRAW(tf);
    // svg.DRAW(tf_trans);
    // svg.EXPORT("test.svg");

    svg_str = "\n<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    svg_str += "\n<defs>";
    svg_str += "\n\t<marker";
    svg_str += "\n\t\tstyle=\"overflow:visible\"";
    svg_str += "\n\t\tid=\"Arrow1Sstart\"";
    svg_str += "\n\t\trefX=\"0.0\"";
    svg_str += "\n\t\trefY=\"0.0\"";
    svg_str += "\n\t\torient=\"auto\">";
    svg_str += "\n\t\t\t<path";
    svg_str += "\n\t\t\t\ttransform=\"scale(0.2) translate(6,0)\"";
    svg_str += "\n\t\t\t\tstyle=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"";
    svg_str += "\n\t\t\t\td=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"";
    svg_str += "\n\t\t\t\t/>";
    svg_str += "\n\t\t</marker>";
    svg_str += "\n</defs>";
    svg_str += "\n<circle cx=\"312.000000\" cy=\"624.000000\" r=\"3.0\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1.0\" />";
    svg_str += "\n<line x1=\"216.000000\" x2=\"600.000000\" y1=\"240.000000\" y2=\"240.000000\" stroke=\"purple\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\" />";
    svg_str += "\n<line x1=\"504.000000\" x2=\"408.000000\" y1=\"528.000000\" y2=\"528.000000\" stroke=\"red\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\" />";
    svg_str += "\n<line x1=\"408.000000\" x2=\"408.000000\" y1=\"432.000000\" y2=\"528.000000\" stroke=\"blue\" stroke-width=\"5.0\" marker-start=\"url(#Arrow1Sstart)\" />";
    svg_str += "\n</svg>";

    REQUIRE(svg.EXPORT() == svg_str);
}




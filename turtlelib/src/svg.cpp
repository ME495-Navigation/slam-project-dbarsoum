#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <fstream>

/// \brief 
turtlelib::CREATE_SVG::CREATE_SVG()
{
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
}

std::string turtlelib::CREATE_SVG::EXPORT()
{
    svg_str += "\n</svg>";
    return svg_str;
}

void turtlelib::CREATE_SVG::EXPORT(std::string filename)
{
    svg_str += "\n</svg>";
    std::ofstream file;
    file.open(filename);
    file << svg_str;
    file.close();
}

void turtlelib::CREATE_SVG::DRAW(turtlelib::Point2D pt, std::string color)
{
    turtlelib::Point2D pixel_pt;
    pixel_pt.x = pt.x *96.0 + 8.5/2.0*96.0;
    pixel_pt.y = -pt.y *96.0 + 11.0/2.0*96.0;
    svg_str += "\n<circle";
    svg_str += " cx=\"" + std::to_string(pixel_pt.x) + "\"";
    svg_str += " cy=\"" + std::to_string(pixel_pt.y) + "\"";
    svg_str += " r=\"3.0\"";
    svg_str += " stroke=\"" + color + "\"";
    svg_str += " fill=\"" + color + "\"";
    svg_str += " stroke-width=\"1.0\"";
    svg_str += " />";
}

void turtlelib::CREATE_SVG::DRAW(turtlelib::Point2D head, turtlelib::Point2D tail, std::string color)
{
    turtlelib::Point2D pixel_pt1;
    turtlelib::Point2D pixel_pt2;
    pixel_pt1.x = head.x *96.0 + 8.5/2.0*96.0;
    pixel_pt1.y = -head.y *96.0 + 11.0/2.0*96.0;
    pixel_pt2.x = tail.x *96.0 + 8.5/2.0*96.0;
    pixel_pt2.y = -tail.y *96.0 + 11.0/2.0*96.0;
    svg_str += "\n<line";
    svg_str += " x1=\"" + std::to_string(pixel_pt1.x) + "\"";
    svg_str += " x2=\"" + std::to_string(pixel_pt2.x) + "\"";
    svg_str += " y1=\"" + std::to_string(pixel_pt1.y) + "\"";
    svg_str += " y2=\"" + std::to_string(pixel_pt2.y) + "\"";
    svg_str += " stroke=\"" + color + "\"";
    svg_str += " stroke-width=\"5.0\"";
    svg_str += " marker-start=\"url(#Arrow1Sstart)\"";
    svg_str += " />";
}

void turtlelib::CREATE_SVG::DRAW(turtlelib::Transform2D tf)
{
    /// head is point 1 and tail is point 2 and point 3
    turtlelib::Point2D pt1;
    turtlelib::Point2D pt2;
    turtlelib::Point2D pt3;
    turtlelib::Point2D pixel_pt1;
    turtlelib::Point2D pixel_pt2;
    turtlelib::Point2D pixel_pt3;
    
    pt1.x = 0.0;
    pt1.y = 0.0;
    pt2.x = 1.0;
    pt2.y = 0.0;
    pt3.x = 0.0;
    pt3.y = 1.0;

    turtlelib::Point2D tf_pt1 = tf(pt1);
    turtlelib::Point2D tf_pt2 = tf(pt2);
    turtlelib::Point2D tf_pt3 = tf(pt3);

    pixel_pt1.x = tf_pt1.x *96.0 + 8.5/2.0*96.0;
    pixel_pt1.y = -tf_pt1.y *96.0 + 11.0/2.0*96.0;
    pixel_pt2.x = tf_pt2.x *96.0 + 8.5/2.0*96.0;
    pixel_pt2.y = -tf_pt2.y *96.0 + 11.0/2.0*96.0;
    pixel_pt3.x = tf_pt3.x *96.0 + 8.5/2.0*96.0;
    pixel_pt3.y = -tf_pt3.y *96.0 + 11.0/2.0*96.0;

    svg_str += "\n<line";
    svg_str += " x1=\"" + std::to_string(pixel_pt2.x) + "\"";
    svg_str += " x2=\"" + std::to_string(pixel_pt1.x) + "\"";
    svg_str += " y1=\"" + std::to_string(pixel_pt2.y) + "\"";
    svg_str += " y2=\"" + std::to_string(pixel_pt1.y) + "\"";
    svg_str += " stroke=\"red\"";
    svg_str += " stroke-width=\"5.0\"";
    svg_str += " marker-start=\"url(#Arrow1Sstart)\"";
    svg_str += " />";

    svg_str += "\n<line";
    svg_str += " x1=\"" + std::to_string(pixel_pt3.x) + "\"";
    svg_str += " x2=\"" + std::to_string(pixel_pt1.x) + "\"";
    svg_str += " y1=\"" + std::to_string(pixel_pt3.y) + "\"";
    svg_str += " y2=\"" + std::to_string(pixel_pt1.y) + "\"";
    svg_str += " stroke=\"blue\"";
    svg_str += " stroke-width=\"5.0\"";
    svg_str += " marker-start=\"url(#Arrow1Sstart)\"";
    svg_str += " />";

}


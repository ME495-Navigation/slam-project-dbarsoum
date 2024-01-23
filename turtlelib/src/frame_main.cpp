#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

/// \brief prompt user to enter two transforms
/// compute and output the composition of the two transforms
using namespace std;

turtlelib::Vector2D normalize(turtlelib::Vector2D v_b)
{
    turtlelib::Vector2D v_bhat;
    double v_mag = sqrt(pow(v_b.x, 2) + pow(v_b.y, 2));
    v_bhat.x = v_b.x / v_mag;
    v_bhat.y = v_b.y / v_mag;
    return v_bhat;
}

int main()
{   
    turtlelib::CREATE_SVG svg;

     /// COLORS
    std::string color_purple = "purple";
    std::string color_brown = "brown";
    std::string color_orange = "orange";

    /// TRANSFORMS
    turtlelib::Transform2D T_ab;
    turtlelib::Transform2D T_bc;
    turtlelib::Transform2D T_ba;
    turtlelib::Transform2D T_cb;
    turtlelib::Transform2D T_ac;
    turtlelib::Transform2D T_ca;

    cout << "Enter transform T_{a,b}:\n";
    cin >> T_ab;

    cout << "Enter transform T_{b,c}:\n";
    cin >> T_bc;

    T_ba = T_ab.inv();
    T_cb = T_bc.inv();
    // T_ac = T_ab * T_bc;
    T_ac = turtlelib::operator*(T_ab, T_bc);
    T_ca = T_ac.inv();

    /// output T_ab, T_bc, and T_ac
    cout << "T_{a,b}: " << T_ab << "\n";
    cout << "T_{b,a}: " << T_ba << "\n";
    cout << "T_{b,c} "  << T_bc << "\n";
    cout << "T_{c,b}: " << T_cb << "\n";
    cout << "T_{a,c}: " << T_ac << "\n";
    cout << "T_{c,a}: " << T_ca << "\n";

    /// draw each frame in the svg file
    svg.DRAW(T_ab);
    svg.DRAW(T_ba);
    svg.DRAW(T_bc);
    svg.DRAW(T_cb);
    svg.DRAW(T_ac);
    svg.DRAW(T_ca);

    /// POINTS
    turtlelib::Point2D p_a;
    turtlelib::Point2D p_b;
    turtlelib::Point2D p_c;

    /// prompt user to enter a point
    cout << "Enter a point:\n";
    cin >> p_a;

    p_b = T_ba.operator()(p_a);
    p_c = T_ca.operator()(p_a);

    // output for points
    cout << "p_a: " << p_a << "\n";
    cout << "p_b: " << p_b << "\n";
    cout << "p_c: " << p_c << "\n";

    svg.DRAW(p_a, color_purple);
    svg.DRAW(p_b, color_brown);
    svg.DRAW(p_c, color_orange);

    // VECTORS
    turtlelib::Vector2D v_b;
    turtlelib::Vector2D v_bhat;
    turtlelib::Vector2D v_a;
    turtlelib::Vector2D v_c;

    /// POINTS (needed for drawing vectors)
    turtlelib::Point2D head;
    turtlelib::Point2D tail;
    tail.x = 0;
    tail.y = 0;

    /// prompt user to enter a vector
    cout << "Enter vector v_b:\n";
    cin >> v_b;

    /// normalize the vector
    v_bhat = normalize(v_b);
    head.x = v_bhat.x;
    head.y = v_bhat.y;
    svg.DRAW(head, tail, color_brown);

    v_a = T_ab(v_b);
    head.x = v_a.x;
    head.y = v_a.y;
    svg.DRAW(head, tail, color_purple);

    v_c = T_cb(v_b);
    head.x = v_c.x;
    head.y = v_c.y;
    svg.DRAW(head, tail, color_orange);

    /// output the vectors
    cout << "v_bhat: " << v_bhat << "\n";
    cout << "v_a: " << v_a << "\n";
    cout << "v_b: " << v_b << "\n";
    cout << "v_c: " << v_c << "\n";

    /// TWISTS
    turtlelib::Twist2D V_b;
    turtlelib::Twist2D V_a;
    turtlelib::Twist2D V_c;
    
    /// prompt user to enter a twist
    cout << "Enter twist V_b:\n";
    cin >> V_b;

    /// compute the twist in frame a
    V_a = T_ba(V_b);
    head.x = V_a.x;
    head.y = V_a.y;
    svg.DRAW(head, tail, color_purple);

    /// output the twist in frame c
    V_c = T_ca(V_a);
    head.x = V_c.x;
    head.y = V_c.y;
    svg.DRAW(head, tail, color_orange);

    cout << "V_a: " << V_a << "\n";
    cout << "V_b: " << V_b << "\n";
    cout << "V_c: " << V_c << "\n";

    /// ouput the drawing to a file
    svg.EXPORT("/tmp/frames.svg");

    return 0;

}

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

/// \brief prompt user to enter two transforms
/// compute and output the composition of the two transforms
using namespace std;
int main()
{   
    turtlelib::CREATE_SVG svg;

    turtlelib::Transform2D T_ab;
    cout << "Enter transform T_{a,b}:\n";
    cin >> T_ab;

    turtlelib::Transform2D T_bc;
    cout << "Enter transform T_{b,c}:\n";
    cin >> T_bc;

    turtlelib::Transform2D T_ba;
    turtlelib::Transform2D T_cb;
    turtlelib::Transform2D T_ac;
    turtlelib::Transform2D T_ca;

    T_ba = T_ab.inv();
    T_cb = T_bc.inv();
    T_ac = T_ab * T_bc;
    T_ca = T_ac.inv();

    /// output T_ab, T_bc, and T_ac
    cout << "T_{a,b}: " << T_ab << "\n";
    cout << "T_{b,a}: " << T_ba << "\n";
    cout << "T_{b,c} "  << T_bc << "\n";
    cout << "T_{c,b}: " << T_cb << "\n";
    cout << "T_{a,c}: " << T_ac << "\n";
    cout << "T_{c,a}: " << T_ca << "\n";

    /// promt user to enter a point
    turtlelib::Point2D p_a;
    cout << "Enter a point:\n";
    cin >> p_a;

    turtlelib::Point2D p_b;
    turtlelib::Point2D p_c;

    p_b = T_ba.operator()(p_a);
    p_c = T_ca.operator()(p_a);

    // output for points
    cout << "p_a: " << p_a << "\n";
    cout << "p_b: " << p_b << "\n";
    cout << "p_c: " << p_c << "\n";

    /// prompt user to enter a vector
    turtlelib::Vector2D v_b;
    cout << "Enter vector v_b:\n";
    cin >> v_b;

    
    /// prompt user to enter a twist
    turtlelib::Twist2D V_b;
    cout << "Enter twist V_b:\n";
    cin >> V_b;

    return 0;

}

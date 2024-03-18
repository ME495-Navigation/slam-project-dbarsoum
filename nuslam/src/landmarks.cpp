#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <sstream>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <armadillo>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_srvs/srv/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "nusim/srv/teleport.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;

class Landmarks : public rclcpp::Node
{
public:
  Landmarks()
  : Node("landmarks")
  {
    // Create a publisher for the landmarks
    landmark_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("landmarks", 10);
    // Create a subscriber for the laser scan
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("laser_scan", 10, std::bind(&Landmarks::laser_callback, this, std::placeholders::_1));

  }
  private:
  //brief Publisher for the landmarks
  // cluster points into groups corresponding to individual landmarks
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // cluster laser scanner points into groups by distance
    // discard clusters with fewer than 3 points
    // for each cluster, compute the centroid and add it to the list of landmarks
    std::vector<std::vector<double>> clusters;
    std::vector<double> cluster;
    for (int i = 0; i < msg->ranges.size(); i++)
    {
      if (msg->ranges[i] < 3.0)
      {
        cluster.push_back(msg->ranges[i]);
      }
      else
      {
        if (cluster.size() > 0)
        {
          clusters.push_back(cluster);
          cluster.clear();
        }
      }
    }
    // circle fitting algorithm
    auto x_centroid = mean_calc(clusters[0]);
    auto y_centroid = mean_calc(clusters[1]);

    // shift coordinates so centroid is at origin
    for (int i = 0; i < clusters.size(); i++)
    {
      clusters[i][0] -= x_centroid;
      clusters[i][1] -= y_centroid;
    }

    // compute the mean of the squared distances from the origin
    double z_i = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
      z_i += std::pow(clusters[i][0], 2) + std::pow(clusters[i][1], 2);
    }
    auto z_bar = z_i / clusters.size();

    // data matrix from the clusters
    arma::mat data(clusters.size(), 4); // Z = data

    for (int i = 0; i < clusters.size(); i++)
    {
      data(i, 0) = std::pow(clusters[i][0], 2) * std::pow(clusters[i][0], 2);
      data(i, 1) = clusters[i][0];
      data(i, 2) = clusters[i][1];
      data(i, 3) = 1;
    }
    // moment matrix
    arma::mat M = data.t() * data; 

    // constraint matrix
    arma::mat H(4, 4);
    H.zeros();
    H(0, 0) = 8 * z_bar;
    H(0, 3) = 2;
    H(1, 1) = 1;
    H(2, 2) = 1;
    H(3, 0) = 2;

    // inv of H
    arma::mat H_inv = arma::inv(H);

    // compute the singular value decomposition of Z
    arma::mat U;
    arma::vec s;
    arma::mat V;
    svd(U, s, V, data);

    auto sigma = arma::diagmat(s);

    // auto sigma = data / (U * V.t());
    auto Z = U * sigma * V.t();

    // singular value sigma_4
    if (s(3) < 10e-12)
    {
      // A equals fourth column matric of V
      A = V.col(3);
    }
    else if (s(3) > 10e-12)
    {
      auto Y = V * sigma * V.t();
      auto Q = Y * H_inv * Y;
      // A* is the eigenvector of Q corresponding to the smallest eigenvalue
      arma::vec eigval;
      arma::mat eigvec;
      eig_sym(eigval, eigvec, Q);

      int min_index = 0;
      double min_value = 1000000.0;
      for (int i = 0; i < eigval.size() && eigval.size() > 0; i++)
      {
        if (eigval(i) < min_value)
        {
          min_value = eigval(i);
          min_index = i;
        }
      }

      arma::vec A_star = arma::zeros(4);
      for (int i = 0; i < 4; i++)
      {
        A_star.at(i) = eigvec(i, min_index);
      }
      // auto A_star = eigvec.col(0);
      A = A_star / Y;
    }

    // compute the radius and center of the circle
    auto R = std::sqrt(std::pow(A(1), 2) + std::pow(A(2), 2) - 4 * A(0) * A(3)) / (4 * std::pow(A(0), 2));
    auto x_center = -A(1) / (2 * A(0));
    auto y_center = -A(2) / (2 * A(0));

    // shift the center back to the original coordinates
    x_center += x_centroid;
    y_center += y_centroid;

    // classify the clusters as circle or not circle
    for (int i = 0; i < clusters.size(); i++)
    {
      // compute the root mean square error of the points from the circle
      auto rmse = 1 / clusters.size() * std::sqrt(std::pow(clusters[i][0] - x_center, 2) + std::pow(clusters[i][1] - y_center, 2) - R);
      if (rmse < 0.1)
      {
        // circle
        // print circle center and radius
        std::cout << "Circle center: (" << x_center << ", " << y_center << ")" << std::endl;
        std::cout << "Circle radius: " << R << std::endl;
        std::cout << "Circle detected " << std::endl;
      }
      else
      {
        // not circle
        std::cout << "Not a circle" << std::endl;
      }
    }

    // create a marker array message
    visualization_msgs::msg::MarkerArray landmarks;

    // for each cluster, compute the centroid and add it to the list of landmarks
    for (int i = 0; i < clusters.size(); i++)
    {
      visualization_msgs::msg::Marker landmark;
      landmark.header.frame_id = "base_link";
      landmark.header.stamp = this->now();
      landmark.ns = "landmarks";
      landmark.id = i;
      landmark.type = visualization_msgs::msg::Marker::SPHERE;
      landmark.action = visualization_msgs::msg::Marker::ADD;
      landmark.pose.position.x = x_center; // i am not sure if this is correct
      landmark.pose.position.y = clusters[i][1] + y_center; // or if this is correct
      landmark.pose.position.z = 0;
      landmark.pose.orientation.x = 0.0;
      landmark.pose.orientation.y = 0.0;
      landmark.pose.orientation.z = 0.0;
      landmark.pose.orientation.w = 1.0;
      landmark.scale.x = 0.1;
      landmark.scale.y = 0.1;
      landmark.scale.z = 0.1;
      landmark.color.a = 1.0;
      landmark.color.r = 0.0;
      landmark.color.g = 0.0;
      landmark.color.b = 1.0;
      landmarks.markers.push_back(landmark);
    }

    // publish the landmark array
    landmark_pub_->publish(landmarks);
  }

  double euclidean_distance(double x1, double y1, double x2, double y2)
  {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
  }

  double mean_calc(std::vector<double> v)
  {
    double sum = 0;
    for (int i = 0; i < v.size(); i++)
    {
      sum += v[i];
    }
    return sum / v.size();
  }

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr landmark_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  arma::vec A = arma::zeros(4);
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Landmarks>());
  rclcpp::shutdown();
  return 0;
}
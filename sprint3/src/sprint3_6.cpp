#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cmath>
#include <vector>
#include <utility>
#include <tf2/utils.h> 
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

/**
 * @brief This cpp file runs the code for 3.6 
 * 
 * This file contains a ROS2 node that subscribes to LaserScan and Odometry data, processes laser scan points to detect 
 * cylindrical objects, and visualizes the detected objects using markers in RViz. It also performs checks to avoid detecting walls 
 * and corners, and only recognizes circular segments that fit the target cylinder size. Key functionalities include:
 * - Subscribing to LaserScan and Odometry data
 * - Processing the scan to identify segments and circular objects
 * - Publishing visualization markers for detected cylinders in RViz
 * 
 * The node uses the following techniques:
 * - Conversion of local laser scan points to global coordinates based on odometry data
 * - Detection of cylinders by analyzing geometric properties of laser scan segments
 * - Visualization of detected segments and cylinders in RViz
 * 
 * @note The file includes utility functions to check for walls, corners, and existing objects to prevent duplicate detection.
 * 
 * @dependencies
 * - rclcpp (ROS2 client library)
 * - sensor_msgs, nav_msgs (ROS2 message types)
 * - tf2 for coordinate frame transformations
 * - visualization_msgs for RViz marker visualization
 * 
 */

class Cylinder : public rclcpp::Node
{
public:

Cylinder() : Node("sprint3_6")
{
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&Cylinder::scanCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Cylinder::odomCallback, this, std::placeholders::_1));
    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/visualization_marker", 10);
}

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

    nav_msgs::msg::Odometry currentOdom;
    bool firstCent;
    std::vector<geometry_msgs::msg::Point> centres;
    int ct_;

void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    auto segments = countSegments(scan);
}
 
void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
    currentOdom = *odom;
}
 
std::vector<std::vector<geometry_msgs::msg::Point>> countSegments(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
    geometry_msgs::msg::Point current, previous;
    std::vector<std::vector<geometry_msgs::msg::Point>> segmentVector;
    std::vector<geometry_msgs::msg::Point> currentSegment;
    bool segStarted = false;
 
    for (size_t i = 1; i < scan->ranges.size(); i++)
    {
        if (std::isinf(scan->ranges.at(i - 1)) || std::isnan(scan->ranges.at(i - 1)))
        {
            continue;
        }
 
        while (i < scan->ranges.size() && (!std::isnan(scan->ranges.at(i))) && (scan->ranges.at(i) < scan->range_max))
        {
            float previousAngle = scan->angle_min + scan->angle_increment * (i - 1);
            previous.x = scan->ranges.at(i - 1) * cos(previousAngle);
            previous.y = scan->ranges.at(i - 1) * sin(previousAngle);
            previous.z = 0;
 
            float currentAngle = scan->angle_min + scan->angle_increment * i;
            current.x = scan->ranges.at(i) * cos(currentAngle);
            current.y = scan->ranges.at(i) * sin(currentAngle);
            current.z = 0;
 
            float dist = hypot((current.x - previous.x), (current.y - previous.y));
 
            if (dist < 0.075)
            {
                if (!segStarted)
                {
                    segStarted = true;
                    currentSegment.clear();
                }
                currentSegment.push_back(localToGlobal(currentOdom, current));
            }
            else
            {
                if (segStarted && !currentSegment.empty())
                {
                    segmentVector.push_back(currentSegment);
                    if (!isThisAWall(currentSegment))
                    {
                        if (!isThisACorner(currentSegment))
                            detectCylinder(currentSegment);
                    }
                    segStarted = false;
                }
                break;
            }
            i++;
        }
 
        if (segStarted && !currentSegment.empty())
        {
            segmentVector.push_back(currentSegment);
            rclcpp::sleep_for(std::chrono::milliseconds(10));
            if (!isThisAWall(currentSegment))
            {
                if (!isThisACorner(currentSegment))
                    detectCylinder(currentSegment);
            }
            segStarted = false;
        }
    }
 
    return segmentVector;
}
 
void detectCylinder(const std::vector<geometry_msgs::msg::Point> &segment)
{
    if (segment.size() >= 6)
    {
        const auto &p1 = segment.front();
        const auto &p2 = segment.at(segment.size() / 2);
        const auto &p3 = segment.back();
        double a = hypot(p2.x - p1.x, p2.y - p1.y);
        double b = hypot(p3.x - p2.x, p3.y - p2.y);
        double c = hypot(p3.x - p1.x, p3.y - p1.y);
        double cosTheta = (a * a + b * b - c * c) / (2 * a * b);
        double theta = acos(cosTheta);
        double R = c / (2 * fabs(sin(theta / 2)));
        double targetRadius = 0.15;
        double tolerance_ = 0.01;
 
        if (fabs(R - targetRadius) < tolerance_)
        {
            geometry_msgs::msg::Point centre = findCentre(segment.front(), segment.back(), targetRadius);
            if (!checkExisting(centre) || firstCent)
            {
                firstCent = false;
                centres.push_back(centre);
                RCLCPP_INFO(this->get_logger(), "Circle with radius ~%.2fm detected. Center: x = %.2f, y = %.2f, z = %.2f", targetRadius, centre.x, centre.y, centre.z);
                visualization_msgs::msg::Marker marker = produceMarkerCylinder(centre);
                marker_pub_->publish(marker);
            }
        }
    }
}
 
geometry_msgs::msg::Point findCentre(geometry_msgs::msg::Point P1, geometry_msgs::msg::Point P2, double r)
{
    geometry_msgs::msg::Point centre;
    double mx = (P1.x + P2.x) / 2.0;
    double my = (P1.y + P2.y) / 2.0;
    double d = sqrt((P2.x - P1.x) * (P2.x - P1.x) + (P2.y - P1.y) * (P2.y - P1.y));
    double h = sqrt(pow(r, 2) - pow(d / 2, 2));
    double dx = -(P2.y - P1.y) / d;
    double dy = (P2.x - P1.x) / d;
    double c1x = mx + h * dx;
    double c1y = my + h * dy;
    double c2x = mx - h * dx;
    double c2y = my - h * dy;
    double d2 = sqrt((currentOdom.pose.pose.position.x - c1x) * (currentOdom.pose.pose.position.x - c1x) + (currentOdom.pose.pose.position.y - c1y) * (currentOdom.pose.pose.position.y - c1y));
    double d3 = sqrt((currentOdom.pose.pose.position.x - c2x) * (currentOdom.pose.pose.position.x - c2x) + (currentOdom.pose.pose.position.y - c2y) * (currentOdom.pose.pose.position.y - c2y));
 
    if (d2 > d3)
    {
        centre.x = c1x;
        centre.y = c1y;
        centre.z = currentOdom.pose.pose.position.z;
    }
    else
    {
        centre.x = c2x;
        centre.y = c2y;
        centre.z = currentOdom.pose.pose.position.z;
    }
 
    return centre;
}
 
bool checkExisting(geometry_msgs::msg::Point centre)
{
    bool is_near_existing_center = false;
 
    for (const auto &existing_centre : centres)
    {
        double distance = sqrt(pow(centre.x - existing_centre.x, 2) + pow(centre.y - existing_centre.y, 2));
        if (distance < 1)
        {
            is_near_existing_center = true;
            break;
        }
    }
 
    return is_near_existing_center;
}
 
void visualizeSegment(const std::vector<geometry_msgs::msg::Point> &segment)
{
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = "map";
    line_marker.header.stamp = this->get_clock()->now();
    line_marker.ns = "cylinder_markers";
    line_marker.id = ct_++;
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.a = 0.8;
    line_marker.color.r = 0.0;
    line_marker.color.g = 1.0;
    line_marker.color.b = 0.0;
 
    for (const auto &point : segment)
    {
        line_marker.points.push_back(point);
    }
 
    marker_pub_->publish(line_marker);
}
 
visualization_msgs::msg::Marker produceMarkerCylinder(geometry_msgs::msg::Point pt)
{
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cylinder_markers";
    marker.id = ct_++;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = pt;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.2;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
 
    return marker;
}
 
geometry_msgs::msg::Point localToGlobal(const nav_msgs::msg::Odometry &global, const geometry_msgs::msg::Point &local)
{
    geometry_msgs::msg::Point pt;
    pt.x = global.pose.pose.position.x + (local.x * cos(tf2::getYaw(global.pose.pose.orientation)) - local.y * sin(tf2::getYaw(global.pose.pose.orientation)));
    pt.y = global.pose.pose.position.y + (local.x * sin(tf2::getYaw(global.pose.pose.orientation)) + local.y * cos(tf2::getYaw(global.pose.pose.orientation)));
    pt.z = 0;
 
    return pt;
}
 
bool isThisAWall(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float dist = hypot(segment.front().x - segment.back().x, segment.front().y - segment.back().y);
    return dist > 0.6;
}
 
bool isThisACorner(const std::vector<geometry_msgs::msg::Point> &segment)
{
    float a = hypot(segment.back().x - segment[segment.size() / 2].x, segment.back().y - segment[segment.size() / 2].y);
    float b = hypot(segment.front().x - segment[segment.size() / 2].x, segment.front().y - segment[segment.size() / 2].y);
    float c = hypot(segment.back().x - segment.front().x, segment.back().y - segment.front().y);
    float cosTheta = (a * a + b * b - c * c) / (2 * a * b);
    float theta = acos(cosTheta) * 180 / M_PI;
 
    return theta < 120;
}

};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Cylinder>());
    rclcpp::shutdown();
    return 0;
}
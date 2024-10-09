#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LaserScanNth : public rclcpp::Node
{
public:
    LaserScanNth()
        : Node("laser_scan_nth")
    {
        //subscriber to the /scan topic of type sensor_msg
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LaserScanNth::scanCallback, this, std::placeholders::_1));

        //publisher of the same type given topic name /nscan
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/nscan", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)

    {
        int n = 5;
//a copy of the scan data needs to be made so when the data is pushed back
//and then published it is in the right place in the vector for the rviz to read.
        auto scan_copy = std::make_shared<sensor_msgs::msg::LaserScan>(*scan);
        std::vector<float> scan_n;

        scan_copy->header = scan->header; 
        scan_copy->angle_min = scan->angle_min;
        scan_copy->angle_max = scan->angle_max;
        scan_copy->angle_increment = scan->angle_increment;
        scan_copy->time_increment = scan->time_increment;
        scan_copy->scan_time = scan->scan_time;
        scan_copy->range_min = scan->range_min;
        scan_copy->range_max = scan->range_max;
//a for loop which goes through everything in the scan and publishes at every nth index
        for(int i = 0; i < scan->ranges.size(); i += n){
            scan_n.push_back(scan->ranges.at(i));
            RCLCPP_INFO(this->get_logger(), "republished scan index: %d", i);
        }
        //mkes sure the data for the nth scan is at the correct angle for said scan
        scan_copy->angle_increment = scan->angle_increment * n;
        scan_copy->ranges = scan_n;
        scan_pub_->publish(*scan_copy);

        
    }

    // int n;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserScanNth>());
    rclcpp::shutdown();
    return 0;
}
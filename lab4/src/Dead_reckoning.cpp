#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>  
#include <random>



//publish cmd_vel topic 

class Dead_reckoning: public rclcpp::Node
{
public:
Dead_reckoning()
    : Node("Dead_reckoning")
    {
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&Dead_reckoning::odomCallBack, this, std::placeholders::_1));
 
        noise_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom_noise", 10);

        cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise_dist(0.0, 0.1);
    }
    
private:
    void odomCallBack(const nav_msgs::msg::Odometry::SharedPtr odom) {
    double current_pose_x = odom->pose.pose.position.x;
    double current_pose_y = odom->pose.pose.position.y;
    
    //may not need this here. may be in public
    // std::random_device rd;
    // std::mt19937 gen(rd());
    // std::normal_distribution<> noise_dist(0.0, 0.1);

    //create noisy version of data    
    double noisy_x = current_pose_x + noise_dist(gen); 
    double noisy_y = current_pose_y + noise_dist(gen);

    //set up copy of odom and push noisy data to is
    auto odom_noise =*odom;
    odom_noise.pose.pose.position.x = noisy_x;
    odom_noise.pose.pose.position.y = noisy_y;

    //publish noisy odom
    noise_pub->publish(odom_noise);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr noise_pub;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;

    std::mt19937 gen;
    std::normal_distribution<> noise_dist;


};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Dead_reckoning>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
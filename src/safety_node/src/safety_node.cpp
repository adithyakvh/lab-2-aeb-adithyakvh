#include "rclcpp/rclcpp.hpp"
/// CHECK: include needed ROS msg type headers and libraries
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cstdio>
#include <vector>
#include <cmath>
// #include <eigen3/Eigen/Dense>
// #include <opencv2/core/eigen.hpp>

using std::placeholders::_1;

class Safety : public rclcpp::Node {
// The class that handles emergency braking


public:
    
    Safety() : Node("safety_node")
    {
        /*
        You should also subscribe to the /scan topic to get the
        sensor_msgs/LaserScan messages and the /ego_racecar/odom topic to get
        the nav_msgs/Odometry messages

        The subscribers should use the provided odom_callback and 
        scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        */

        /// TODO: create ROS subscribers and publishers
        // std::cout<<"IAMHERE"<<std::endl;
        RCLCPP_INFO(this->get_logger(), "Safety node started");
        ego_car = this->create_subscription<nav_msgs::msg::Odometry>("/ego_racecar/odom", 10, std::bind(&Safety::drive_callback, this, _1));
        scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("scan", 10, std::bind(&Safety::scan_callback, this, _1));
        speed_pub = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

    
        // geometry_msgs::msg::TwistWithCovariance tw_cov;
        // auto tw_cov = std::make_shared<geometry_msgs::msg::Twist>();
        // std::cout<<"EGO"<<std::endl;
        // nav_msgs::msg::Odometry tw_odom;

        // x = 
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_car;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr speed_pub;

    geometry_msgs::msg::TwistWithCovariance tw_cov;
    nav_msgs::msg::Odometry::ConstSharedPtr tw_msg;

    // tw_cov = ego_car;

        



private:
    double speed = 0.0;
    /// TODO: create ROS subscribers and publishers
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_car;



    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        /// TODO: update current speed
        // std::cout<<"Ranges"<<msg.ranges<<std::endl;  
        // std::cout<<"Enter here"<<std::endl;  
        // printf("SPEED %f \n", msg->twist.twist.linear.x);
        tw_cov = msg->twist;


    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /// TODO: calculate TTC
        float start_angle = scan_msg->angle_min;
        float end_angle = scan_msg->angle_max;
        float angle_increments = scan_msg->angle_increment;
        float range_max = scan_msg->range_max;
        std::vector<float> vect_range = scan_msg->ranges;

    
        auto ackermann_message = ackermann_msgs::msg::AckermannDriveStamped();


        // 180 * (x) / pi
        for(int i=0; i<vect_range.size(); i++){
            
            float theta = start_angle + i * angle_increments;
            if (theta < 45.0 && theta > -45.0){
                float r = vect_range[i];
            
                if (std::isnan(r)){
                    r = 0;
                }

                if (std::isinf(r)){
                    r = range_max;
                }
                float r_dot = tw_cov.twist.linear.x * cos(theta);
                if (r_dot <=0.0){
                    r_dot = 0.0;
                }

                float ttc = r / r_dot;

                if (ttc < 2.0){
                    printf("Range is %f \n", r);

                    printf("TTC is %f \n", ttc);

                    ackermann_message.drive.speed = 0;
                    speed_pub->publish(ackermann_message);

                }

            }
            
            
        }

        
        // std::cout<<"TYPE"<<typeid(scan_msg->ranges).name()<<std::endl;
        

        /// TODO: publish drive/brake message
    }

    // rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr speed_pub;


};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
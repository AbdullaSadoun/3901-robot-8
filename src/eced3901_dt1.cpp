/*
Code for DT1
V. Sieben
Version 1.0
Date: Feb 4, 2023
License: GNU GPLv3
*/

// Include important C++ header files that provide class
// templates for useful operations.
#include <chrono>        // Timer functions
#include <functional>    // Arithmetic, comparisons, and logical operations
#include <memory>        // Dynamic memory management
#include <string>        // String functions
#include <cmath>

// ROS Client Library for C++
#include "rclcpp/rclcpp.hpp"
 
// Message types
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

// Create the node class named SquareRoutine
// It inherits rclcpp::Node class attributes and functions
class SquareRoutine : public rclcpp::Node
{
  public:
    // Constructor creates a node named Square_Routine. 
    SquareRoutine() : Node("Square_Routine")
    {
        // Create the subscription
        // The callback function executes whenever data is published to the 'odom' topic.
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&SquareRoutine::topic_callback, this, _1));
          
        // Create the publisher
        // Publisher to a topic named "cmd_vel". The size of the queue is 10 messages.
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
      
        // Create the timer
        timer_ = this->create_wall_timer(100ms, std::bind(&SquareRoutine::timer_callback, this));     
    }

  private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_now = msg->pose.pose.position.x;
        y_now = msg->pose.pose.position.y;
        
        // Convert quaternion to roll-pitch-yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, yaw_now);

        //RCLCPP_INFO(this->get_logger(), "Odom Acquired.");
    }
    
    void timer_callback()
    {
        geometry_msgs::msg::Twist msg;

        // State machine to handle the sequence of movements
        switch (state_)
        {
            case 0: // Move forward
                if (distance_traveled() < 1.0) {
                    msg.linear.x = 0.2; // Move forward
                    msg.angular.z = 0;
                } else {
                    state_ = 1; // Transition to turning
                    reset_distance();
                    
                }
                break;

            case 1: // Turn 90 degrees counterclockwise
            	RCLCPP_INFO(this->get_logger(), "Current Yaw: %.2f", angle_rotated());
                if (angle_rotated() < (M_PI_2)) {
                 //   rclcpp::sleep_for(500ms);
                    msg.linear.x = 0;
                    msg.angular.z = 0.3; // Turn
                } else {
                    state_ = 0; // Transition to moving forward
                    reset_angle();
                    count_++;
                }
                break;

            default:
                msg.linear.x = 0; // Stop
                msg.angular.z = 0;
                break;
        }

        // Stop after completing the square
        if (count_ >= 4) {
            msg.linear.x = 0;
            msg.angular.z = 0;
        }

        publisher_->publish(msg);        
    }
    
    double distance_traveled() const {
        return std::sqrt(std::pow(x_now - x_init, 2) + std::pow(y_now - y_init, 2));
    }

    double angle_rotated() const {
        return std::fabs(yaw_now - yaw_init);
    }

    void reset_distance() {
        x_init = x_now;
        y_init = y_now;
    }

    void reset_angle() {
        yaw_init = yaw_now;
    }

    // Declaration of subscription_ attribute
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
         
    // Declaration of publisher_ attribute      
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    
    // Declaration of the timer_ attribute
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Declaration of Class Variables
    double x_now = 0, x_init = 0, y_now = 0, y_init = 0;
    double yaw_now = 0, yaw_init = 0;
    size_t count_ = 0;
    int state_ = 0;
};
        
//------------------------------------------------------------------------------------
// Main code execution
int main(int argc, char * argv[])
{
    // Initialize ROS2
    rclcpp::init(argc, argv);
  
    // Start node and callbacks
    rclcpp::spin(std::make_shared<SquareRoutine>());
 
    // Stop node 
    rclcpp::shutdown();
    return 0;
}

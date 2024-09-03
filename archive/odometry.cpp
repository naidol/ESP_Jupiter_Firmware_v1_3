#include "odometry.h"
#include <tf2/LinearMath/Quaternion.h>
#include <micro_ros_utilities/type_utilities.h>

// Constructor: initializes the odometry publisher and variables
Odometry::Odometry(rclc_publisher_t* odom_publisher, double wheel_radius, double wheel_separation)
    : odom_publisher_(odom_publisher), wheel_radius_(wheel_radius), wheel_separation_(wheel_separation), x_(0.0), y_(0.0), theta_(0.0) {

    // Initialize the odometry message fields
    odom_msg_.header.frame_id = micro_ros_string_utilities_set("odom");
    odom_msg_.child_frame_id = micro_ros_string_utilities_set("base_link");
}

// Function to update the robot's odometry based on wheel velocities
void Odometry::update(double front_left_wheel_velocity, double front_right_wheel_velocity, double back_left_wheel_velocity, double back_right_wheel_velocity, double dt) {
    
    // Average front and back velocities for each side
    double left_wheel_velocity = (front_left_wheel_velocity + back_left_wheel_velocity) / 2.0;
    double right_wheel_velocity = (front_right_wheel_velocity + back_right_wheel_velocity) / 2.0;

    // Convert wheel velocities to linear and angular velocities of the robot
    double linear_velocity = wheel_radius_ * (left_wheel_velocity + right_wheel_velocity) / 2.0;
    double angular_velocity = wheel_radius_ * (right_wheel_velocity - left_wheel_velocity) / wheel_separation_;

    // Update the robot's pose (x, y, theta)
    double delta_theta = angular_velocity * dt;
    double delta_x = linear_velocity * cos(theta_ + delta_theta / 2.0) * dt;
    double delta_y = linear_velocity * sin(theta_ + delta_theta / 2.0) * dt;

    x_ += delta_x;
    y_ += delta_y;
    theta_ += delta_theta;

    // Create a quaternion from theta for the orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);

    // Fill in the odometry message
    odom_msg_.header.stamp = rclc_now(&odom_msg_.header.stamp); // Replace with your timestamp logic
    odom_msg_.pose.pose.position.x = x_;
    odom_msg_.pose.pose.position.y = y_;
    odom_msg_.pose.pose.orientation.x = q.x();
    odom_msg_.pose.pose.orientation.y = q.y();
    odom_msg_.pose.pose.orientation.z = q.z();
    odom_msg_.pose.pose.orientation.w = q.w();
    
    // Set the velocity in the odom message
    odom_msg_.twist.twist.linear.x = linear_velocity;
    odom_msg_.twist.twist.angular.z = angular_velocity;

    // Publish the odometry message
    rclc_publisher_publish(odom_publisher_, &odom_msg_, NULL);
}

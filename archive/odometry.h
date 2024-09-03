#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <micro_ros_utilities/string_utilities.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

class Odometry {
public:
    Odometry(rclc_publisher_t* odom_publisher, double wheel_radius, double wheel_separation);

    // Function to update the robot's odometry based on wheel encoder readings
    void update(double front_left_wheel_velocity, double front_right_wheel_velocity, double back_left_wheel_velocity, double back_right_wheel_velocity, double dt);

private:
    rclc_publisher_t* odom_publisher_;

    double x_;       // Robot's x position
    double y_;       // Robot's y position
    double theta_;   // Robot's orientation

    double wheel_radius_;
    double wheel_separation_;

    nav_msgs__msg__Odometry odom_msg_;
};

#endif // ODOMETRY_H

#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <iostream>
#include <cstdlib>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define THRE_DIST 0.3


class RobotController {

private:

    ros::NodeHandle node_handle;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    double obstacle_distance;
    float left_distance;
    float front_distance;
    float right_distance;

    // PID control
    float old_prop_error;
    float integral_error;
    
    float target_value = THRE_DIST;
    float KP = 10.0;
    float KI = 0.0;
    float KD = 0.0;
    float time_interval = 0.1;

    bool robot_lost;
    int lost_counter;

    geometry_msgs::Twist calculateCommand();


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    float calculateGain(float value);
    void calculateRobotLost();

public:

    RobotController();
    void run();

};

#endif /** ROBOTCONTROLLER_H **/
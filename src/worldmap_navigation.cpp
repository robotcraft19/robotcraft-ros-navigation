#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

#define THRE_DIST 0.3


class RobotController
{
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

    geometry_msgs::Twist calculateCommand()
    {   
        calculateRobotLost();

        auto msg = geometry_msgs::Twist();
        
        if (front_distance < THRE_DIST) 
        {
            // Prevent robot from crashing
            msg.angular.z = 1.0;
            msg.linear.x = -0.05;
        } 
        else if (robot_lost == true){
            // Robot is lost, go straight to find wall
            msg.linear.x = 0.5;
        }  
        else 
        {
        float gain = calculateGain(right_distance);
        msg.linear.x = 0.5;
        msg.angular.z = gain;
        }

        return msg;
    }


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        // Calculate array size of ranges
        int ranges_len = (msg->angle_max - msg->angle_min) / msg->angle_increment;
        int split_size = ranges_len / 3;

        obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        right_distance = *std::min_element(msg->ranges.begin(), msg->ranges.begin()+split_size);
        front_distance = *std::min_element(msg->ranges.begin()+split_size, msg->ranges.begin()+2*split_size);
        left_distance = *std::min_element(msg->ranges.begin()+2*split_size, msg->ranges.begin()+ranges_len);
    }


    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        // Read theta value of robot
        float robot_theta = msg->pose.pose.orientation.w;
    }

    float calculateGain(float value) {
        float error = this->target_value - value;
        float new_der_err = error - this->old_prop_error;
        float new_int_err = integral_error + error;

        float gain = this->KP*error + this->KI*new_int_err*this->time_interval
            + this->KD*new_der_err/this->time_interval;

        this->old_prop_error = error;
        this->integral_error = new_int_err;         

        return gain;
    }

    void calculateRobotLost() {
        // Calculations needed to check if robot is lost
        if (front_distance > THRE_DIST && right_distance > THRE_DIST 
                && left_distance > THRE_DIST)
        {
            ++lost_counter;
            if (lost_counter >= 75)
                robot_lost = true;
        } 
        else if(front_distance < THRE_DIST || right_distance < THRE_DIST)
        {
            robot_lost = false;
            lost_counter = 0;
        }
    }



public:
    RobotController(){
        // Initialize ROS
        this->node_handle = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = node_handle.subscribe("base_scan", 10, &RobotController::laserCallback, this);

        // Create a subscriber for robot's odometry
        this->odom_sub = node_handle.subscribe("odom", 10, &RobotController::odomCallback, this);
    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10); // Update rate of 10Hz
        while (ros::ok())
        {
            // Calculate the command to apply
            auto msg = calculateCommand();

            // Publish the new command
            this->cmd_vel_pub.publish(msg);

            ros::spinOnce();

            // And throttle the loop
            loop_rate.sleep();
        }
    }

};


int main(int argc, char **argv){
    // Initialize ROS
    ros::init(argc, argv, "robot_controller");

    // Create our controller object and run it
    auto controller = RobotController();
    controller.run();

    // And make good on our promise
    return 0;
}
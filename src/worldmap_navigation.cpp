#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"

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

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        if((left_distance < 0.5 && front_distance > 0.5)) {
            msg.linear.x = 1.0;
        } 
        else if(left_distance > 0.5) {
            msg.angular.z = 1.0;
            msg.linear.x = 0.1;
        }
        else if(front_distance < 0.5) {
            msg.angular.z = -1.0;
            msg.linear.x = -0.1;
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
        
        ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
        ROS_INFO("Min distance left: %f", left_distance);
        ROS_INFO("Min distance front: %f", front_distance);
        ROS_INFO("Min distance right: %f", right_distance);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) 
    {
        // Read theta value of robot
        float theta = msg->pose.pose.orientation.w;
        ROS_INFO("Theta: %f", theta);

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
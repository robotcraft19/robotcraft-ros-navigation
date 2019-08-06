#include <iostream>

#include <cstdlib>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"


class ReactiveController
{
private:
    ros::NodeHandle n;
    ros::Publisher cmd_vel_pub;
    ros::Subscriber laser_sub;

    double obstacle_distance;
    bool robot_stopped;

    geometry_msgs::Twist calculateCommand()
    {
        auto msg = geometry_msgs::Twist();
        
        if(obstacle_distance > 0.5){
            msg.linear.x = 1.0;
        }else{
            // TODO  
        }
        
        return msg;
    }


    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        ROS_INFO("Min distance to obstacle: %f", obstacle_distance);
    }


public:
    ReactiveController(){
        // Initialize ROS
        this->n = ros::NodeHandle();

        // Create a publisher object, able to push messages
        this->cmd_vel_pub = this->n.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        // Create a subscriber for laser scans 
        this->laser_sub = n.subscribe("base_scan", 10, &ReactiveController::laserCallback, this);

    }

    void run(){
        // Send messages in a loop
        ros::Rate loop_rate(10);
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
    ros::init(argc, argv, "reactive_controller");

    // Create our controller object and run it
    auto controller = ReactiveController();
    controller.run();

    // And make good on our promise
    return 0;
}
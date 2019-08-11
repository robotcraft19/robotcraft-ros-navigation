#include "robot_controller.h"

geometry_msgs::Twist RobotController::calculateCommand() 
{   
    calculateRobotLost();

    auto msg = geometry_msgs::Twist();
        
    if (front_distance < THRESHOLD_DISTANCE) 
    {
        // Prevent robot from crashing
        msg.angular.z = 1.0;
        msg.linear.x = -0.05;
    } 
    else if (robot_lost == true)
    {
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

void RobotController::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
    // Calculate array size of ranges
    int ranges_len = (msg->angle_max - msg->angle_min) / msg->angle_increment;
    int split_size = ranges_len / 3;

    obstacle_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    right_distance = *std::min_element(msg->ranges.begin(), msg->ranges.begin()+split_size);
    front_distance = *std::min_element(msg->ranges.begin()+split_size, msg->ranges.begin()+2*split_size);
    left_distance = *std::min_element(msg->ranges.begin()+2*split_size, msg->ranges.begin()+ranges_len);
}

float RobotController::calculateGain(float value) 
{
    float error = this->target_value - value;
    float new_der_err = error - this->old_prop_error;
    float new_int_err = this->integral_error + error;

    float gain = this->KP*error + this->KI*new_int_err*this->time_interval
                 + this->KD*new_der_err/this->time_interval;

    this->old_prop_error = error;
    this->integral_error = new_int_err;         

    return gain;
}

void RobotController::calculateRobotLost() 
{
    // Calculations needed to check if robot is lost
    if (front_distance > THRESHOLD_DISTANCE && right_distance > THRESHOLD_DISTANCE 
        && left_distance > THRESHOLD_DISTANCE) 
    {
            ++lost_counter;

            if (lost_counter >= 75) robot_lost = true;
    } 
    else if(front_distance < THRESHOLD_DISTANCE || right_distance < THRESHOLD_DISTANCE) 
    {
            robot_lost = false;
            lost_counter = 0;
    }
}

RobotController::RobotController() 
{
    // Initialize ROS
    this->node_handle = ros::NodeHandle();

    // Create a publisher object, able to push messages
    this->cmd_vel_pub = this->node_handle.advertise<geometry_msgs::Twist>("cmd_vel", 5);

    // Create a subscriber for laser scans 
    this->laser_sub = node_handle.subscribe("base_scan", 10, &RobotController::laserCallback, this);
}

void RobotController::run()
{
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
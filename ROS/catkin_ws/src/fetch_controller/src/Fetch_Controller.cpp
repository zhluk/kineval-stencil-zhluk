#include "Fetch_Controller.hpp"
#include <cmath>


Fetch_Controller::Fetch_Controller(ros::NodeHandle &nh)
{
    nh_ = nh;

    //TODO: initialize a subscriber that is set to the channel "/base_scan". Set its callback function to be Laser_Scan_Callback
    //TODO: initialize a publisher that is set to the channel "/cmd_vel"

    publisher_ = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    subscriber_ = nh.subscribe("/base_scan", 1000, &Fetch_Controller::Laser_Scan_Callback, this);

}

void Fetch_Controller::Laser_Scan_Callback(const sensor_msgs::LaserScan::ConstPtr &msg_laser_scan)
{
    /*TODO: 
    Given the incoming laser scan message, find the minimium distance of the front facing scans
    Hint: The laser scan measuring directly in front of the robot will be the scan at the middle of the array laser scans. 
    So for finding the minimum, we will ONLY consider the 120 laser scans in the middle of the array of laser scans. 
    If the minimum scan in this direction is greater than 1m, drive forward. 
    Otherwise, turn left. 
    */
    
    // Get the index of measurement at the center of scan
    int index_center = round(msg_laser_scan->ranges.size()/2);

    // Assume the first element is the minimal value
    float min_range = msg_laser_scan->ranges[index_center-60];

    for (int i = 0; i < 120; i++) {
        if (msg_laser_scan->ranges[index_center-60+i] < min_range) {
            min_range = msg_laser_scan->ranges[index_center-60+i];
        }
    }

    std::cout << "Minimum distance: " << min_range << std::endl;

    // Create a Twist message and set its value
    geometry_msgs::Twist msg;

    // If the minimum scan in this direction is greater than 1m, drive forward Otherwise, turn left.
    if (min_range > 1.0) {
        msg.linear.x = 0.5;
        std::cout << "Forward" << std::endl;
    } else {
        msg.angular.z = 1.0;
        std::cout << "Turn" << std::endl;
    }
    
    // Publish the Twist message
    publisher_.publish(msg);

}

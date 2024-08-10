#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>
#include <vector>

class ReactiveNavigation {
public:
    ReactiveNavigation() : obstacle_distance(100.0), robot_stopped(false) {
        ros::NodeHandle nh;
        laser_sub = nh.subscribe("base_scan", 1, &ReactiveNavigation::laser_cb, this);
        cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        rate = ros::Rate(5);  // 5 Hz
    }

    void laser_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
        laser_msg = msg;
    }

    void calculate_command() {
        if (!laser_msg) {
            return; 
        }

        obstacle_distance = *std::min_element(laser_msg->ranges.begin(), laser_msg->ranges.end());

        if (obstacle_distance > 0.6) {
            cmd_vel.linear.x = 1;
            cmd_vel.angular.z = 0.0;
        } else {
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 1.0;
        }

        cmd_vel_pub.publish(cmd_vel);
        obstacle_distance = 100.0;  
    }

    void run() {
        while (ros::ok()) {
            calculate_command();
            rate.sleep();
        }
    }

private:
    geometry_msgs::Twist cmd_vel;
    bool robot_stopped;
    double obstacle_distance;
    sensor_msgs::LaserScan::ConstPtr laser_msg;
    ros::Subscriber laser_sub;
    ros::Publisher cmd_vel_pub;
    ros::Rate rate;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "reactive_navigation");
    try {
        ReactiveNavigation controller;
        controller.run();
    } catch (const ros::Exception& e) {
        ROS_ERROR("%s", e.what());
    }
    return 0;
}


#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_action/ControLaw.h>
#include <tiago_action/lawAction.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core/operations.hpp>
#include <tiago_action/utilities.h>
#include <tiago_action/scan_clusterizer.h>


class ControLaw {
protected:
    ros::NodeHandle nh0_;
    actionlib::SimpleActionServer<tiago_action::lawAction> as0_;
    std::string action_name;
    tiago_action::lawFeedback feedback0_;
    tiago_action::lawResult result0_;
    ros::Publisher pub;
    bool done = false;

public:
    ControLaw(std::string name) : as0_(nh0_, name, boost::bind(&ControLaw::moveCorridor, this, _1), false),
                                  action_name(name) {
        as0_.start();
        pub = nh0_.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 1);
    }

    ~ControLaw(void) {};

    /// function that moves the robot along the corridor
    /// \param goal to be reached (in our case the end of the corridor)
    void moveCorridor(const tiago_action::lawGoalConstPtr& goal);

    /// Gets the current position of the robot from the odometry with the appropriate transfrom and sends a feedback
    /// to the action server.
    void GetRobotPosition();

    };










#endif
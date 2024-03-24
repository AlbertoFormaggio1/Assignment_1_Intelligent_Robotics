#ifndef SRC_ROBOT_H
#define SRC_ROBOT_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <tiago_action/scan_clusterizer.h>
#include <tiago_action/robotAction.h>
#include <tiago_action/lawAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

class RobotActionServer{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<tiago_action::robotAction> as_;
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac_;
    actionlib::SimpleActionClient<tiago_action::lawAction> ac0_;

    std::string action_name_;
    tiago_action::robotFeedback feedback_;
    tiago_action::robotResult result_;

    const float WORLD_X = - 6.579991;
    const float WORLD_Y = 1.369981;

public:
    /// Enumerator used to give the client information about the current state.
    /// The enumerator was used to provide the client more flexibility about the messages to print while being also
    /// intuitive (instead of returning simply numbers).
    enum RobotState{
        MOVING_ROOM,
        MOVING_CORRIDOR,
        START_OF_CORRIDOR,
        END_OF_CORRIDOR,
        GOAL_REACHED,
        DETECTING,
        FAIL,
    };

    RobotActionServer(std::string name): as_(nh_, name, boost::bind(&RobotActionServer::moveAndDetect, this, _1), false), action_name_(name), ac_("move_base", true), ac0_("out_corridor",true){
        ROS_INFO("Server starting...");
        as_.start();
    }

    /// Moves the robot in the narrow corridor
    /// \param goal_0 goal set at the end of the corridor to be reached by the robot when controlled manually
    /// \return true if the robot reached the end of the corridor
    bool moveCorridor(tiago_action::lawGoal goal_0);

    /// function that given a goal returns true or false whether the robot reaches it or not
    /// \param goal point to be reached expressed in coordinates (x,y,th)
    /// \return
    bool moveRobot(const tiago_action::robotGoalConstPtr &goal);

    /// Detects the obstacles clusterizing the point obtained by the lidar and keeping only the cylindrical objects
    void detectObstacles();

    /// function that calls moveRobot() and detectObstacles()
    /// \param goal the goal given by the action_client
    void moveAndDetect(const tiago_action::robotGoalConstPtr &goal);

    /// Get the robot position in the global reference frame with (x,y) coordinates
    /// \param x output param x
    /// \param y output param y
    void GetRobotPosition (float& x, float& y);

    /// Publish a feedback with the current robot position
    void publishRobotPosition();

    /// Given the x, y, th from the robot's odometry, returns the position of the robot in the global reference frame defined
    /// in Gazebo.
    /// \param x odometry x
    /// \param y odometry y
    /// \param th odometry th
    /// \param out_x x in Gazebo reference frame
    /// \param out_y y in Gazebo reference frame
    void convertReferenceFrame(float x, float y, float th, float &out_x, float &out_y);

    /// Returns to the client the information on robot's motion given the feedback from the control law server
    /// \param feedback control law feedback
    void publishControlLawFeedback(const tiago_action::lawFeedbackConstPtr& feedback);

    void convertReferenceFrame(float x, float y, float th, float dx, float dy, float &out_x, float &out_y);


        ~RobotActionServer(void){};
    };


#endif //SRC_ROBOT_H

//
// Created by sara on 11/12/23.
//

#include "../include/tiago_action/Robot.h"
#include <tf/transform_listener.h>
#include <tiago_action/ControLaw.h>

// rostopic pub -1 /move_base_simple/goal geo
// metry_msgs/PoseStamped
// '{header: {stamp: now, frame_id: "map"},
// pose: {position: {x: 10.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}'
bool RobotActionServer::moveRobot(const tiago_action::robotGoalConstPtr &goal) {
    // wait for the server of the "/move_base" topic to start
    ac_.waitForServer();

    // Set the position in the map to achieve as goal
    move_base_msgs::MoveBaseGoal g;

    g.target_pose.header.frame_id = "map";
    g.target_pose.header.stamp = ros::Time::now();
    g.target_pose.pose.position.x = goal->x;
    g.target_pose.pose.position.y = goal->y;

    tf2::Quaternion q;
    q.setRPY(0, 0, goal->th); // roll=0, pitch=0, yaw
    q.normalize();
    g.target_pose.pose.orientation.x = q.getX();
    g.target_pose.pose.orientation.y = q.getY();
    g.target_pose.pose.orientation.z = q.getZ();
    g.target_pose.pose.orientation.w = q.getW();

    ROS_INFO("Sending goal");

    // sending goal to the server
    ac_.sendGoal(g);

    // The robot is moving, send this information to the client
    feedback_.status = MOVING_ROOM;
    RobotActionServer::publishRobotPosition();

    // it waits for the results
    ac_.waitForResult(ros::Duration(45.0));

    //gets the result
    actionlib::SimpleClientGoalState result = ac_.getState();

    // Checks if the robot was able to successfully reach the final position or not. Returns the state accordingly.
    if (result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The robot achieved the final position with success");
        feedback_.status = GOAL_REACHED;
        publishRobotPosition();
        return true;
    } else {
        ROS_INFO("The robot could not find a way to reach its goal");
        feedback_.status = FAIL;
        publishRobotPosition();
        return false;
    }
}

bool RobotActionServer::moveCorridor(tiago_action::lawGoal goal_0) {

    // Wait for the law server to start
    ac0_.waitForServer();

    goal_0.distance = 0.8;

    ROS_INFO("Sending goal");
    feedback_.status = START_OF_CORRIDOR;
    publishRobotPosition();

    ac0_.sendGoal(goal_0, NULL, NULL, boost::bind(&RobotActionServer::publishControlLawFeedback, this, _1));

    ac0_.waitForResult(ros::Duration(45.0));

    actionlib::SimpleClientGoalState result = ac0_.getState();

    if (result.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("The robot exited the narrow passage!");
        feedback_.status = END_OF_CORRIDOR;
        publishRobotPosition();
        return true;
    } else {
        ROS_INFO("The robot couldn't exit the narrow passage");
        feedback_.status = FAIL;
        return false;
    }
}

void RobotActionServer::detectObstacles() {
    // Vector containing the coordinates of the points read by the LiDAR
    std::vector<cv::Point2f> coordinates_scanner;
    // Wait for the first reading of the laser scanner on the /scan topic
    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("scan", nh_);
    // Read the coordinates and convert them into cartesian coordinates (the result is in coordinates_scanner)
    ScannerClusterizer::read_scanner(msg->ranges, coordinates_scanner, msg->angle_min, msg->angle_increment);

    std::vector<cv::Point2f> coordinates_objects;
    // We compute the number and the location of the cylindrical objects
    ROS_INFO("Detecting objects...");
    int n_objects = ScannerClusterizer::compute_clusters(coordinates_scanner, coordinates_objects, 0.3);
    ROS_INFO("Number of clusters is %d", n_objects);
    result_.num_objects = n_objects;

    // Use the odometry of the robot to find its location with respect to the global reference frame
    float xr, yr;
    RobotActionServer::GetRobotPosition(xr, yr);

    // Convert the location of the objects from the robot reference frame to the global reference frame
    std::vector<float> coordinates_glob_x(coordinates_objects.size());
    std::vector<float> coordinates_glob_y(coordinates_objects.size());
    std::vector<float> coordinates_rob_x;
    std::vector<float> coordinates_rob_y;
    for (int i = 0; i < coordinates_objects.size(); i++) {

        tf::TransformListener listener;
        tf::StampedTransform transform;
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
        float th = tf::getYaw(transform.getRotation());
        float x, y;


        convertReferenceFrame(coordinates_objects[i].x, coordinates_objects[i].y, th, transform.getOrigin().x(), transform.getOrigin().y(), x,y);
        ROS_INFO("x: %f, y: %f", x, y);

        convertReferenceFrame(x, y, 0, RobotActionServer::WORLD_X, RobotActionServer::WORLD_Y, coordinates_glob_x[i],coordinates_glob_y[i]);

        //coordinates_glob_x.push_back(coordinates_objects[i].x + xr);
        //coordinates_glob_y.push_back(coordinates_objects[i].y + yr);
        coordinates_rob_x.push_back(coordinates_objects[i].x);
        coordinates_rob_y.push_back(coordinates_objects[i].y);

        ROS_INFO("the world reference is X=[%f], Y= [%f]", coordinates_glob_x[i], coordinates_glob_y[i]);
        ROS_INFO("Object %d = %f %f", i, coordinates_objects[i].x, coordinates_objects[i].y);
    }

    // Set the coordinates as the result of the action to be then read by the server
    result_.object_positions_x_glob = coordinates_glob_x;
    result_.object_positions_y_glob = coordinates_glob_y;
    result_.object_positions_x_rob = coordinates_rob_x;
    result_.object_positions_y_rob = coordinates_rob_y;
}

void RobotActionServer::moveAndDetect(const tiago_action::robotGoalConstPtr &goal) {
    tiago_action::lawGoal goal_0;

    ROS_INFO("Starting the motion ");
    bool got_out_corridor = moveCorridor(goal_0);

    if (!got_out_corridor) {
        as_.setAborted(result_);
        return;
    }

    ROS_INFO("Heading towards the final goal");
    bool succeded = moveRobot(goal);

    // If there was a problem in the navigation of the robot set the action result to ABORTED and return.
    if (!succeded) {
        as_.setAborted(result_);
        return;
    }

    detectObstacles();

    as_.setSucceeded(result_);
}


void RobotActionServer::publishRobotPosition() {
    float x;
    float y;
    RobotActionServer::GetRobotPosition(x, y);
    feedback_.x = x;
    feedback_.y = y;
    as_.publishFeedback(feedback_);
}

void RobotActionServer::GetRobotPosition(float &x, float &y) {
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        // Wait for a transform from base_link (robot reference frame) to odom (global reference frame)
        listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(1.0));
        listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
        // Get the coordinates of thfeedbacke position (0,0) of the robot in the global reference frame
        float x_rob = transform.getOrigin().x();
        float y_rob = transform.getOrigin().y();
        // Get the yaw of the rotation of the transform
        float th = tf::getYaw(transform.getRotation());
        ROS_INFO("%f", th);

        convertReferenceFrame(x_rob, y_rob, th, RobotActionServer::WORLD_X, RobotActionServer::WORLD_Y, x, y);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

void RobotActionServer::convertReferenceFrame(float x, float y, float th, float dx, float dy, float &out_x, float &out_y) {

    float trans[4][4] = {{cos(th), -sin(th), 0, dx},
                         {sin(th), cos(th),  0, dy},
                         {0,       0,        1, 0},
                         {0,       0,        0, 1}};

    float coord[4] = {x, y, 0, 1};

    std::vector<float> res(4);

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            res[i] = res[i]+(trans[i][j] * coord[j]);
        }
    }

    out_x=res[0];
    out_y=res[1];
}

void RobotActionServer::publishControlLawFeedback(const tiago_action::lawFeedbackConstPtr& feedback){
    feedback_.status = MOVING_CORRIDOR;
    ROS_INFO("the coordinates BEFORE the transformation are X= [%f], Y=[%f]",feedback->x,feedback->y);
    float x; float y;
    convertReferenceFrame(feedback->x, feedback->y, feedback->th, RobotActionServer::WORLD_X, RobotActionServer::WORLD_Y,x, y);
    ROS_INFO("the coordinates AFTER the transformation are X= [%f], Y=[%f]",x,y);
    feedback_.x = x;
    feedback_.y = y;

    as_.publishFeedback(feedback_);
}

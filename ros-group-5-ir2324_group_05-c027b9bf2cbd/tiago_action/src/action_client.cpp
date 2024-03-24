#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <tiago_action/robotAction.h>
#include "tiago_action/Robot.h"


void doneCb(const actionlib::SimpleClientGoalState& state, const tiago_action::robotResultConstPtr& res){
    printf("Finished in state [%s]\n", state.toString().c_str());

    printf("------------------ DETECTION RESULTS ---------------------\n");

    // If the action was successful, print the result of the detection
    if(state.state_ == actionlib::SimpleClientGoalState::SUCCEEDED) {
        printf("Number of objects is: %d\n", res->num_objects);
        printf("The location of the objects is:\n");
        for (int i = 0; i < res->num_objects; i++) {
            printf("Object %d:\nGlobal Reference Frame: x = %.3f ; y = %.3f\nRobot Reference Frame: x = %.3f ; y = %.3f\n",
                     i, res->object_positions_x_glob[i], res->object_positions_y_glob[i],
                     res->object_positions_x_rob[i], res->object_positions_y_rob[i]);
        }
    }
}

void activeCb(){
    printf("------------------ ACTION STARTED ------------------\n");
}

void feedbackCb(const tiago_action::robotFeedbackConstPtr & feedback){
    switch (feedback->status) {
        case RobotActionServer::MOVING_ROOM:
            printf("The robot is moving in the room\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        case RobotActionServer::MOVING_CORRIDOR:
            printf("The robot is moving in the corridor\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        case RobotActionServer::GOAL_REACHED:
            printf("The robot has reached the desired goal!\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        case RobotActionServer::DETECTING:
            printf("The robot is detecting the objects in the room\n");
            break;
        case  RobotActionServer::FAIL:
            printf("The robot has failed, goal not reached\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        case RobotActionServer::START_OF_CORRIDOR:
            printf("The robot started to navigate in the narrow corridor\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        case RobotActionServer::END_OF_CORRIDOR:
            printf("The robot reached the end of the narrow corridor\n");
            printf("The position of the robot is X = [%f], Y = [%f]\n", feedback->x, feedback->y);
            break;
        default: ROS_INFO ("Something went wrong, incorrect parameter was used\n");

    }

}


int main(int argc, char** argv){
    ros::init(argc, argv, "action_client");
    ros::NodeHandle n;

    if(argc != 4){
        ROS_ERROR("Correct usage: client x y theta. DEBUG: Try 10.8 -0.2 -0.4");
        return 1;
    }

    actionlib::SimpleActionClient<tiago_action::robotAction> ac("move_around",true);
    printf("Waiting for action server to start.\n");
    ac.waitForServer(); //will wait for infinite time until the server starts

    printf("Action server started, sending goal.\n");
    tiago_action::robotGoal goal;
    // Set the order attribute in the goal and send it to the server

    goal.x = std::atof(argv[1]);
    goal.y = std::atof(argv[2]);
    goal.th = std::atof(argv[3]); //Theta in radiants

    //the goal is sent to the server
    ac.sendGoal(goal, &doneCb, &activeCb, &feedbackCb);

    //it waits for the result up till 90 seconds
    bool finished_before_timeout = ac.waitForResult(ros::Duration(90.0));

    // If the robot took more than  1.30 minutes to reach the goal, assume that the server had an error and thus
    // it wasn't able to successfully reach the goal
    if (!finished_before_timeout) {
        printf("Action did not finish before the time out.");
        return 1;
    }

    return 0;
}

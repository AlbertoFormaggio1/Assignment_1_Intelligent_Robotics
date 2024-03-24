#include <tiago_action/ControLaw.h>
#include <tf/transform_listener.h>
#include <tiago_action/Robot.h>

    void ControLaw::moveCorridor(const tiago_action::lawGoalConstPtr &goal) {

        ros::Rate loop_rate(5);
        geometry_msgs::Twist adjust_pose;
        feedback0_.x = feedback0_.y = 0;
        float min=-1;

        int j = 1;
        ROS_INFO("Starting motion");
        if (!ControLaw::done) {
            while (ros::ok() && min <= goal->distance) {
                // Set the navigation to done
                done=true;
                // we wait for the message from scan
                sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan",
                                                                                                          nh0_);
                ROS_INFO("Scan message obtained");

                int start_index = 20;

                //we redefine the min angle
                float max_angle = msg->angle_max - start_index * msg->angle_increment;

                // we look for the min ro value from the vector given by the scan
                min = msg->range_max;
                int min_index = -1;
                for (int i = start_index; i < msg->ranges.size() - start_index; i++) {
                    if (msg->ranges[i] < min) {
                        min = msg->ranges[i];
                        min_index = i - start_index;
                    }
                }
                float teta_rel = max_angle - (min_index) * msg->angle_increment;


                //if the robot is too close to the wall it will adjust its pose
                if (min <= 0.40) {
                    ROS_INFO("Adjusting pose");

                    if (teta_rel > 0) {
                        ROS_INFO("Going left");
                        adjust_pose.angular.z = 0.75;
                        adjust_pose.linear.x = 0.075;
                        pub.publish(adjust_pose);
                        ros::spinOnce();
                        loop_rate.sleep();
                    } else if (teta_rel < 0) {
                        ROS_INFO("Going right");
                        adjust_pose.angular.z = -0.75;
                        adjust_pose.linear.x = 0.075;
                        pub.publish(adjust_pose);
                        ros::spinOnce();
                        loop_rate.sleep();
                    }

                } else {
                    ROS_INFO("Heading straight");
                    adjust_pose.linear.x = 0.2;
                    adjust_pose.angular.z = 0;
                    pub.publish(adjust_pose);
                    ros::spinOnce();
                    loop_rate.sleep();

                    j++;
                    // Send a feedback to the client every 10 readings
                    if (j % 10 == 0) {
                        GetRobotPosition();
                        ROS_INFO(" The position of the robot is X=[%f], Y=[%f] ", feedback0_.x, feedback0_.y);

                        j = 1;
                    }

                    //ROS_INFO(" The position of the robot is X=[%f], Y=[%f] ", feedback0_.x,feedback0_.y);
                    ROS_INFO(" The min distance is [%f] ", min);
                    ROS_INFO(" The min distance (goal) is [%f] ", goal->distance);
                }

            }
            adjust_pose.linear.x = 0;
            adjust_pose.angular.z = 0;
            pub.publish((adjust_pose));
            ros::spinOnce();

            result0_.position = true;
            ROS_INFO("The path was successful");
            as0_.setSucceeded(result0_);
        } else { ROS_INFO("No narrow passage has been found");

        }
    }

void ControLaw::GetRobotPosition() {
    tf::TransformListener listener;
    tf::StampedTransform transform;

    try {
        listener.waitForTransform("odom","base_link",ros::Time(0),ros::Duration(1.0));
        listener.lookupTransform("odom","base_link",ros::Time(0),transform);
        feedback0_.x = transform.getOrigin().x();
        feedback0_.y = transform.getOrigin().y();
        feedback0_.th = tf::getYaw(transform.getRotation());
        as0_.publishFeedback(feedback0_);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

int main(int argc, char **argv){

    ros::init(argc,argv, "server0");
    ControLaw law("out_corridor");
    ros::spin();
    return 0;
};
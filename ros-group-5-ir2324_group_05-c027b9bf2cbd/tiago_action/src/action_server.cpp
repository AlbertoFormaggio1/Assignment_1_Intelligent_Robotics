#include <navfn/navfn_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include "../include/tiago_action/Robot.h"
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <iostream>
#include <fstream>


int main(int argc, char** argv){
    ros::init(argc, argv, "server");

    ROS_INFO("Before server starts...");
    RobotActionServer robot("move_around");

    ros::spin();
}



/*
 * PRINT IMAGE
 *
 *  sensor_msgs::ImagePtr message = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
        std::ofstream myfile ("example.txt");
        if (myfile.is_open())
        {
            for(int count = 0; count < message->data.size(); count ++){
                if(count % message->width == 0)
                    myfile << "\n";
                if(message->data[count] == 255)
                    myfile << "1";
                else
                    myfile << "0";
            }
            myfile.close();
        }
 *
 *
 */
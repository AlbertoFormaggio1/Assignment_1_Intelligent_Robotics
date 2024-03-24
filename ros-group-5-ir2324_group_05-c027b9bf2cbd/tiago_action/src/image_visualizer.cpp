#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <image_transport/image_transport.h>

void messageCallback(const sensor_msgs::ImageConstPtr& msg){
    try
    {
        ROS_INFO("Printing image");
        cv::imwrite("file.jpg", cv_bridge::toCvShare(msg, "mono8")->image);
    }
    catch (cv_bridge::Exception& e)
   {
     ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
   }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "charging_station");
    ros::NodeHandle n;
    //cv::namedWindow("view");

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("image_topic", 1, messageCallback);

    ros::spin();
    //cv::destroyAllWindows();
}
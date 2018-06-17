/**
 * @file   ohm_rrl_start_stop.cpp
 * @author Johanna Gleichauf
 * @date   Stand 16.06.2018
 *
 *
 */
#include <cstdlib>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <std_srvs/SetBool.h>


image_transport::Publisher pubSwitch;
image_transport::Subscriber imgSub;
image_transport::Subscriber imgNew;
bool param = false;

//Callback function to republish image_raw topic as switch topic
void callCam(const sensor_msgs::ImageConstPtr& camImage)
{
  std::cout << "callback camera " << std::endl;
  pubSwitch.publish(camImage);

}

bool service(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res)
{
  ros::NodeHandle nh_ser;
  ros::NodeHandle private_nh_ser("~");

  image_transport::ImageTransport it_ser(nh_ser);
  std::cout << "service bool " << std::endl;
  std::cout << "param val before service "  << param << std::endl;
  param = req.data;

  std::cout << "param val after service "  << param << std::endl;

  if(param)
    imgSub.shutdown();
  else
    //create new subscriber
    imgSub = it_ser.subscribe("/usb_cam/image_raw", 1, callCam);
  return true;
}


/** @function main */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  image_transport::ImageTransport it(nh);
  sensor_msgs::ImagePtr msg;

  //Subscriber /usb_cam/image_raw
  imgSub = it.subscribe("/usb_cam/image_raw", 1, callCam);
//  imgNew = it.subscribe("/usb_cam/image_raw", 1, callCam);
  pubSwitch = it.advertise("/switch", 1);
  ros::ServiceServer serv= nh.advertiseService("BoolService", service);




  ros::spin();
}




/*
 * QrCodeDetection.h
 *
 *  Created on: Mar 24, 2015
 *      Author: jon
 */

#ifndef QRCODEDETECTION_H_
#define QRCODEDETECTION_H_

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <zbar.h>
#include <ros/ros.h>
#include <ohm_perception_msgs/QrArray.h>
#include <ohm_perception_msgs/Qr.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

class QrCodeDetection {
public:
  QrCodeDetection(void);
  virtual ~QrCodeDetection(void);

  void imageCallBack(const sensor_msgs::ImageConstPtr& imageRos);

private:
  ros::NodeHandle                 _nh;
  image_transport::ImageTransport _it;

  ros::Publisher _qrPub;

  image_transport::Publisher  _imagePub;
  image_transport::Subscriber _imageSubs;

  ohm_perception_msgs::QrArray _qrArray;

  cv::Mat _frame;
  zbar::ImageScanner _scanner;
};

#endif /* QRCODEDETECTION_H_ */

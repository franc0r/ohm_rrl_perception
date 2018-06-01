/*
 * QrCodeDetection.h
 *
 *  Created on: Mar 24, 2015
 *      Author: jon
 */

#ifndef QRCODEDETECTION_H_
#define QRCODEDETECTION_H_

#include "opencv2/opencv.hpp"
#include <zbar.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>

struct QrCode
{
  std::string label;
  std::vector<cv::Point> corners;
};

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

//  ohm_perception_msgs::QrArray _qrArray;

  std::vector<QrCode> _qrs;

  cv::Mat            _frame;
  zbar::ImageScanner _scanner;
};

#endif /* QRCODEDETECTION_H_ */

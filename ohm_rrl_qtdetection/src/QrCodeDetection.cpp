/*
 * QrCodeDetection.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: jon
 */

#include "QrCodeDetection.h"
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>

namespace enc = sensor_msgs::image_encodings;



QrCodeDetection::QrCodeDetection(void) :
    _it(_nh) {
  ros::NodeHandle prvNh("~");

  std::string qrTopic;
  std::string camPubTopic;
  std::string camSubTopic;

  prvNh.param("qr_topic",             qrTopic,     std::string("/qr/pose"));
  prvNh.param("cam_publisher_topic",  camPubTopic, std::string("image/qr_detection_left"));
  prvNh.param("cam_subscriber_topic", camSubTopic, std::string("/image_raw"));

  _qrPub     = _nh.advertise<ohm_perception_msgs::QrArray>(qrTopic, 2);
  _imageSubs = _it.subscribe(camSubTopic, 1, &QrCodeDetection::imageCallBack, this);
  _imagePub  = _it.advertise(camPubTopic, 2);

  _scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

QrCodeDetection::~QrCodeDetection() {
  // TODO Auto-generated destructor stub
}
void QrCodeDetection::imageCallBack(const sensor_msgs::ImageConstPtr& imageRos) {
  cv_bridge::CvImagePtr cv_ptr;

  try {
    cv_ptr = cv_bridge::toCvCopy(imageRos, enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }

  _frame = cv_ptr->image;

  if (_imagePub.getNumSubscribers() > 0) {

    cv::Mat grey;
    cv::cvtColor(_frame, grey, CV_BGR2GRAY);

    zbar::Image image(grey.cols, grey.rows, "Y800", grey.data, grey.cols * grey.rows);
    const int n = _scanner.scan(image);

    std::vector<std::string> qr_text;

    //create the QrArray
    _qrArray.qr.clear();
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      std::vector<cv::Point> corners;
      for (unsigned int i = 0; i < symbol->get_location_size(); i++)
        corners.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));

      const cv::RotatedRect rotRect = minAreaRect(corners);
      for (unsigned int i = 0; i < 4; i++) {
        cv::line(_frame, corners[i], corners[(i + 1) % 4], cv::Scalar(0, 0, 255), 3);
      }

      cv::Point qrCenter;
      qrCenter = (corners[0] + corners[2]) * 0.5;

      ohm_perception_msgs::Qr qr;
      qr.id   = ohm_perception_msgs::Qr::NONE;
      qr.u    = qrCenter.x;
      qr.v    = qrCenter.y;
      qr.data = symbol->get_data();
      _qrArray.qr.push_back(qr);

      qr_text.push_back(symbol->get_data());
    }

    // clean up
    image.set_data(NULL, 0);

    //publish QrArray and the image
    if (_qrArray.qr.size()) _qrPub.publish(_qrArray);

    const cv::Point imageCenter(320, 240);
    for (unsigned int i = 0; i < _qrArray.qr.size(); i++)
    {
      cv::Point qrCenter;
      qrCenter.x = _qrArray.qr.at(i).u - 20;
      qrCenter.y = _qrArray.qr.at(i).v + 10;

      cv::putText(_frame, qr_text[i], qrCenter, cv::FONT_HERSHEY_SIMPLEX,
          1.2, cvScalar(0,255,0), 2, CV_AA);
      //cv::line(_frame, imageCenter, cv::Point(_qrArray.qr.at(i).u, _qrArray.qr.at(i).v), cv::Scalar(0, 255, 0), 3);
      std::cout << "found qr: " << qr_text[i] << std::endl;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg();
    _imagePub.publish(msg);
  }
}

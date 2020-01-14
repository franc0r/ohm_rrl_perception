/*
 * QrCodeDetection.cpp
 *
 *  Created on: Mar 24, 2015
 *      Author: jon
 */

#include "QrCodeDetection.h"

namespace enc = sensor_msgs::image_encodings;

using namespace std;
using namespace zbar;
using namespace cv;

QrCodeDetection::QrCodeDetection()
      : _width(0), _height(0), _it(_nh)
{
   ros::NodeHandle prvNh("~");
   std::string cali_path;
   std::string qrTopic;
   std::string camPubTopic;
   std::string camSubTopic;

   int width = 0;
   int height = 0;
   double rateVar = 0.0;

   prvNh.param("qr_topic", qrTopic, std::string("/qr/pose"));
   prvNh.param("cam_publisher_topic", camPubTopic, std::string("image/qr_detection_left"));
   prvNh.param("cam_subscriber_topic", camSubTopic, std::string("/image")); //original: /image_raw

   prvNh.param<double>("rate", rateVar, 5.0);
   prvNh.param<int>("cam_width", width, 640);
   prvNh.param<int>("cam_height", height, 480);

   _width = width;
   _height = height;
   _cali_path = cali_path;

   _rate = new ros::Rate(rateVar);
   _qrPub = _nh.advertise<ohm_rrl_perception_msgs::QrArray>(qrTopic, 2);
   _imageSubs = _it.subscribe(camSubTopic, 1, &QrCodeDetection::imageCallBack, this);
   _imagePub  = _it.advertise(camPubTopic, 2);

   _scanner.set_config(ZBAR_QRCODE, ZBAR_CFG_ENABLE, 1);
}

QrCodeDetection::~QrCodeDetection()
{
   // TODO Auto-generated destructor stub
}
void QrCodeDetection::imageCallBack(const sensor_msgs::ImageConstPtr& imageRos)
{
   cv_bridge::CvImagePtr cv_ptr;

   try {
      cv_ptr = cv_bridge::toCvCopy(imageRos, enc::BGR8);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
   }


   _frame = cv_ptr->image;

   Mat grey;

   cvtColor(_frame, grey, CV_BGR2GRAY);

   Image image(grey.cols, grey.rows, "Y800", grey.data, grey.cols * grey.rows);
   int n = _scanner.scan(image);


   std::vector<std::string> qr_text;

   //create the QrArray
   _qrArray.qr.clear();
   for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
      std::vector<cv::Point> corners;
      for (unsigned int i = 0; i < symbol->get_location_size(); i++)
         corners.push_back(cv::Point(symbol->get_location_x(i), symbol->get_location_y(i)));

      cv::RotatedRect rotRect = minAreaRect(corners);
      for (unsigned int i = 0; i < 4; i++) {
         line(_frame, corners[i], corners[(i + 1) % 4], Scalar(255, 0, 0), 2);
      }

      cv::Point qrCenter;
      qrCenter = (corners[0] + corners[2]) * 0.5;

      ohm_rrl_perception_msgs::Qr qr;
      qr.id = ohm_rrl_perception_msgs::Qr::NONE;
      qr.u = qrCenter.x;
      qr.v = qrCenter.y;
      qr.data = symbol->get_data();
      _qrArray.qr.push_back(qr);

      qr_text.push_back(symbol->get_data());
   }

   // clean up
   image.set_data(NULL, 0);

   //publish QrArray and the image
   if (_qrArray.qr.size())
      _qrPub.publish(_qrArray);

   if (_imagePub.getNumSubscribers()) {
      cv::Point imageCenter(320, 240);
      for (unsigned int i = 0; i < _qrArray.qr.size(); i++) {
         cv::Point qrCenter;
         qrCenter.x = _qrArray.qr.at(i).u - 20;
         qrCenter.y = _qrArray.qr.at(i).v + 10;

         cv::putText(_frame, qr_text[i], qrCenter, FONT_HERSHEY_SIMPLEX, 1.0, Scalar(0, 256, 0), 1, 8, false);
         cv::line(_frame, imageCenter, cv::Point(_qrArray.qr.at(i).u, _qrArray.qr.at(i).v), cv::Scalar(256, 0, 256), 1);
      }

      sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", _frame).toImageMsg();
      _imagePub.publish(msg);
   }
}

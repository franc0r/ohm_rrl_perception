/*
 * MotionDetectionNode.h
 *
 *  Created on: 20.08.2013
 *      Author: chris
 */

#ifndef MOTIONDETECTIONNODE_H_
#define MOTIONDETECTIONNODE_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv/opencv.hpp>
//
#include <opencv2/video/background_segm.hpp>

//#include <ohm_perception_msgs/Rect.h>
#include <dynamic_reconfigure/server.h>


#include <ohm_rrl_motiondetection/MotionDetectionConfig.h>


/**
 * @class       MotionDetectionNode
 * @author      Christian Pfitzner
 * @date        2013-08-21
 */
class MotionDetectionNode
{
public:
  /**
   * Function to receive instance of node
   * @return    instance of node
   */
  static MotionDetectionNode* getInstance(void);
  /**
   * Destructor
   */
  ~MotionDetectionNode(void);
  /**
   * Function to spin ros node
   */
  void run(void);
  /**
   * Function to set threshold for motion detection
   * @param     th      theshold
   */
  void setThreshold(const int th)       { _threshold = th;  this->updateConfig(); }
  /**
   * Function to set numbers of frames
   * @param     frames  numbers of frames
   */
  void setFrames(const int frames)      { _frames = frames; this->updateConfig(); }
  /**
   * Function to update parameters via dynamic reconfigure parameters
   * @param config
   */
  void setConfig(const ohm_motiondetection::MotionDetectionConfig& config) {_config = config; }

private:
  /**
   * Private Constructor for singleton pattern
   */
  MotionDetectionNode(void);
  /**
   * Private Copy Constructor for singleton pattern
   * @param
//   */
  MotionDetectionNode( const MotionDetectionNode & ) = delete;
  /**
   * Callback function for image
   * @param     img     image message
   */
  void callbackImage(const sensor_msgs::ImageConstPtr& img);
  /**
   * Funtion to reconfigure theshold and frame rate
   */
  void updateConfig(void);

  /**
   * Function to calculate contours
   * @param img
   * @return
   */
  cv::Mat calculateContours(const sensor_msgs::ImageConstPtr& img);
  /**
   * Function to calculate labels based on contours
   * @param pts
   * @param labels
   * @return
   */
  unsigned int calculateLabels(std::vector<cv::Point>& pts, std::vector<int>& labels);
  /**
   * Function to calculate label markers
   * @param lbl
   * @param pts
   * @param labels
   * @param n_labels
   */
  void calculateLabelMarkers(cv::Mat3b& lbl, std::vector<cv::Point>& pts, std::vector<int>& labels, unsigned int n_labels);
  /**
   * Function to calculate rectangles for visualization
   * @param frame
   */
  void calculateRectangles(cv::Mat frame);




  ros::NodeHandle                      _nh;             //!< Ros node handle
  static MotionDetectionNode*          _instance;       //!< instance of node

  image_transport::ImageTransport      _it;             //!< image tansport
  image_transport::Subscriber          _img_sub;        //!< subscriber to image topic
  image_transport::Publisher           _img_pub;

  ros::Publisher                       _notify_pub;     //!< publisher for notification rect
  ros::ServiceServer                   _motion_serv;    //!< publisher for motion detection
  ros::Publisher                       _motion_pub;     //!< publisher for motion detection

  cv::Ptr<cv::BackgroundSubtractorMOG2>_bg;             //!< background substractor
  std::vector<cv::Vec4i>               _hierarchy;
  std::vector<std::vector<cv::Point> > _contours;       //!< contours for motion detection

 // ohm_perception_msgs::Rect            _rect;


  cv::Point _centroid;                                  //!< centroid of motion
  cv::Rect  _box;                                       //!< bounding box around motion

  int _frames;
  int _threshold;

  bool _motionDetected;                                  //!< true if motion detected
  bool _checkMotion;                                     //!< true for motion checking
  bool _debug;                                           //!< true for debugging

  ohm_motiondetection::MotionDetectionConfig _config;

};

// init singleton pattern
MotionDetectionNode* MotionDetectionNode::_instance = 0;


#endif /* MOTIONDETECTIONNODE_H_ */

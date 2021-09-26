/**
 * @file    MotionDetectionNode.h
 * @author  Niklas Kohlisch, kohlischni71082@th-nuernberg.de, previous version Christian Pfitzner
 * @brief   Header for MotionDetectionNode.h
 * @version 2.0
 * @date    2021-09-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef MOTIONDETECTIONNODE_H_
#define MOTIONDETECTIONNODE_H_

#include <ros/ros.h> 
#include <vector>
#include <image_transport/image_transport.h> 
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <dynamic_reconfigure/server.h>


#include <ohm_rrl_motiondetection2/MotionDetectionConfig.h>


/**
 * @class       MotionDetectionNode
 * @author      Christian Pfitzner, edited by Niklas Kohlisch
 * @date        2021-09-26
 * 
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
   * Function to update parameters via dynamic reconfigure parameters
   * @param config
   */
  void setConfig(const ohm_rrl_motiondetection2::MotionDetectionConfig& config) {_config = config; }

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
   * Function to calculate contours
   * @param img
   * @return
   */
  cv::Mat calculateContours(const sensor_msgs::ImageConstPtr& img); 

  ros::NodeHandle                      _nh;             //!< Ros node handle
  static MotionDetectionNode*          _instance;       //!< instance of node

  image_transport::ImageTransport      _it;             //!< image tansport
  image_transport::Subscriber          _img_sub;        //!< subscriber to image topic
  image_transport::Publisher           _img_pub;


  std::vector<cv::Vec4i>               _hierarchy;
  std::vector<std::vector<cv::Point> > _contours;       //!< contours for motion detection
  std::vector<cv::Mat>                 _buffer;

  cv::Point _centroid;                                  //!< centroid of motion
  cv::Rect  _box;                                       //!< bounding box around motion

  bool _motionDetected;                                  //!< true if motion detected
  bool _debug;                                           //!< true for debugging

  ohm_rrl_motiondetection2::MotionDetectionConfig _config;

};

// init singleton pattern
MotionDetectionNode* MotionDetectionNode::_instance = 0;


#endif /* MOTIONDETECTIONNODE_H_ */

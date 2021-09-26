/**
 * @file    MotionDetectionNode.cpp
 * @author  Niklas Kohlisch kohlischni71082@th-nuernberg.de, Christian Pfitzner
 * @brief   Motion detection using edge difference
 * @details Does: gaussian blur -> grayscale conversion -> clahe -> sobel edge detection
 * -> comparison between at least 2 up to 10 frames -> threshold filter -> dilate -> erode 
 * -> contour size filtering -> textual output for detected motion, optional polynomial contours,
 * rectangles. Uses dynamic reconfiure to set parameters for filters, presets included.
 * @version 2.0
 * @date    2021-09-26
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#include "MotionDetectionNode.h"
#include <dynamic_reconfigure/server.h>
#include <string>
#include <sstream>
#include <iostream>

namespace         enc        = sensor_msgs::image_encodings;
static const char WINDOW[]   = "Image window";
auto              font       = cv::FONT_HERSHEY_PLAIN;
auto              font_scale = 1.0;
cv::Scalar        red(0 ,0,255);
cv::Scalar        green(0,255, 0);
cv::Point         place_text(10, 20);

MotionDetectionNode* MotionDetectionNode::getInstance(void)
{
  if(!_instance) _instance = new MotionDetectionNode();
  return(_instance);
}

MotionDetectionNode::~MotionDetectionNode(void)
{
  if(_instance) {
    delete _instance;
    _instance = 0;
  }
}

void MotionDetectionNode::run(void)
{
//  while(ros::ok())
    ros::spin();
}


//################# PRIVATE #######################
MotionDetectionNode::MotionDetectionNode(void)
    : _it(_nh)
, _motionDetected(false)
, _debug(false)
{
  ros::NodeHandle private_nh("~");

  // Parameters for launch file
  std::string image_topic;
  std::string motion_image_topic;
  std::string rect_topic;

  private_nh.param("image_topic",         image_topic,         std::string("image_raw"));
  private_nh.param("motion_image",        motion_image_topic,  std::string("image/motion"));
  private_nh.param("debug",               _debug,              bool(false));


  _img_sub        = _it.subscribe(image_topic,        1, &MotionDetectionNode::callbackImage, this);
  _img_pub        = _it.advertise(motion_image_topic, 1);
}

void MotionDetectionNode::callbackImage(const sensor_msgs::ImageConstPtr& img)
{

  // do nothing if nobody subscribes to the node
   if(_img_pub.getNumSubscribers() == 0)  return;

   cv::Mat frame = this->calculateContours(img);

   if (_contours.size() > 0)
   {
      _motionDetected = true;
   }


    cv_bridge::CvImage output;
    output.header   = img->header;
    output.encoding = enc::BGR8;
    output.image    = frame;
    _img_pub.publish(output.toImageMsg());

}


cv::Mat MotionDetectionNode::calculateContours(const sensor_msgs::ImageConstPtr& img)
{
   cv_bridge::CvImagePtr cv_ptr;
   try {
     cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
   }
   catch (cv_bridge::Exception& e) {
     ROS_ERROR("cv_bridge exception: %s", e.what());
   }

   cv::Mat     src(cv_ptr->image);
   cv::Mat     blurred, equalized, gray, edges;

  /* Remove noise and unnecsessary detail */
  const auto kernel_size = _config.bluring_factor*2 + 1;
  cv::GaussianBlur(src, blurred, cv::Size(kernel_size,kernel_size),kernel_size,kernel_size);

  /* Convert to grayscale and enhance contrast */
  cv::cvtColor(blurred, gray, cv::COLOR_BGR2GRAY);
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
  clahe->setClipLimit(_config.clahe_clip_limit);
  clahe->setTilesGridSize(cv::Size(_config.clahe_grid_size, _config.clahe_grid_size));
  clahe->apply(gray, equalized);


  double scale = _config.sobel_scale;
  int delta = 0;
  int ddepth = CV_16S;

  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  cv::Sobel(equalized, grad_x, ddepth, 1, 0, 2 *_config.sobel_kernel_size -1, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  cv::Sobel( equalized, grad_y, ddepth, 0, 1, 2 *_config.sobel_kernel_size -1, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  cv::addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges );

  cv::Mat diff, comp, thresh;
  

  if(_buffer.size() == _config.history)
  {
    double weight = 0.5;
    cv::subtract(edges, _buffer.back(), comp);  // difference between actual and previous frame
    if(_debug){
      cv::imshow("Difference_to_previous_frame", comp);
      cv::waitKey(30);
    }
    for(int i = _buffer.size() - 1; i > 0; --i) // differences in further history
    {
      cv::subtract(edges, _buffer[i-1], diff);  // difference between previous frames and actual frame
      cv::addWeighted(comp, weight, diff, 1-weight, 0, comp); // combine differences with weights
      weight *= 0.5;                                          // the older the frame, the less weigth it gets
    }
    if(_debug){
      cv::imshow("Weighted_sum_of_differences_in_history", comp);
      cv::waitKey(30);
    }
    /* Reduce background noise via threshold */
    auto cluster_distance_threshold = _config.noise_reduction;
    cv::threshold(comp, thresh, cluster_distance_threshold, 255,cv::THRESH_BINARY);
    if(_debug){
      cv::imshow("Filtered_output_after_applying_threshold", thresh);
      cv::waitKey(30);
    }

    _buffer.push_back(edges);       // store latest frame in buffer
    _buffer.erase(_buffer.begin()); // remove one frame from buffer (oldest)

    /* Connect structures */
    cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(_config.merge_contours, _config.merge_contours));
    cv::dilate(thresh, thresh, kernel_erode);
    cv::erode( thresh, thresh, kernel_erode);
    if(_debug){
      cv::imshow("Output_after_dilating_and_eroding", thresh);
      cv::waitKey(30);
    }

    cv::findContours(thresh, _contours, _hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE,  cv::Point(0,0));
    // remove contors with small size
    std::vector<std::vector<cv::Point> > filtered_contours;
    for(unsigned int i=0 ; i<_contours.size() ; ++i)
    {
      if(_contours[i].size() > _config.min_contour_size)
        filtered_contours.push_back(_contours[i]);
    }

    /// Approximate contours to polygons + get bounding rects and circles
    std::vector<std::vector<cv::Point> > contours_poly( filtered_contours.size() );
    std::vector<cv::Rect> boundRect( filtered_contours.size() );
    std::vector<cv::Point2f>center( filtered_contours.size() );
    std::vector<float>radius( filtered_contours.size() );

    for( int i = 0; i < filtered_contours.size(); i++ )
    { 
      cv::approxPolyDP( cv::Mat(filtered_contours[i]), contours_poly[i], 3, true );
      boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );
    }

    /// Draw polygonal contour + bonding rects
    for( int i = 0; i< filtered_contours.size(); i++ )
    {
      // Draw polygonal contours
      if(_config.show_contours)
        cv::drawContours( src, contours_poly, i, green, 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
      // Draw rectangles
      if(_config.show_rectangles)
        cv::rectangle( src, boundRect[i].tl(), boundRect[i].br(), red, 2, 8, 0 );          
    }
    if(filtered_contours.size() > 0)
      cv::putText(src, "Motion detected!", place_text, font, font_scale, red, 1, cv::LINE_AA);

  }
  else if(_buffer.size() > _config.history)
    _buffer.erase(_buffer.begin());
  else
    _buffer.push_back(edges);

  return src;
}
/**
 * @brief Enumerator for presets in reconfigure
 */
enum presets {
  rotating_disks,
  wibbely_wobbely_strings,
  far_away,
  vibration_tolerant
};

/**
 * @brief Copy configuration and set presets if selected
 * @param config 
 * @param level 
 */
void callbackConfig(ohm_rrl_motiondetection2::MotionDetectionConfig &config, uint32_t level)
{
  ROS_INFO("Updated config via dynamic reconfigure");
  MotionDetectionNode::getInstance()->setConfig(config);
  if(level){
    switch((presets)config.preset)
    {
      case rotating_disks: 
        ROS_INFO("rotating_disks");
        config.sobel_kernel_size = 0;
        config.sobel_scale       = 0.56;
        config.history           = 2;
        config.noise_reduction   = 57;
        config.bluring_factor    = 1;
        config.min_contour_size  = 15;
        config.merge_contours    = 19;
        config.clahe_clip_limit  = 2;
        config.clahe_grid_size   = 8;
        break;
      case wibbely_wobbely_strings: 
        ROS_INFO("strings"); 
        config.sobel_kernel_size = 0;
        config.sobel_scale       = 1;
        config.history           = 2;
        config.bluring_factor    = 1;
        config.noise_reduction   = 100;
        config.merge_contours    = 18;
        config.min_contour_size  = 17;
        config.clahe_clip_limit  = 1.7;
        config.clahe_grid_size   = 6;
        break;
      case far_away: 
        ROS_INFO("far away"); 
        config.sobel_kernel_size = 0;
        config.sobel_scale       = 0.6;
        config.history           = 7;
        config.noise_reduction   = 100;
        config.bluring_factor    = 0;
        config.min_contour_size  = 10;
        config.merge_contours    = 11;
        config.clahe_clip_limit  = 1.7;
        config.clahe_grid_size   = 7;
        break;
      case vibration_tolerant: 
        ROS_INFO("vibration"); 
        break;
    }
  }
}




/*
 * Main program
 */
int main(int argc,char **argv)
{
   ros::init(argc, argv, "MotionDetection");
   ROS_INFO_STREAM("Starting motion detection node");


   // initialize dynamic reconfigure
   dynamic_reconfigure::Server<ohm_rrl_motiondetection2::MotionDetectionConfig> srv;
   dynamic_reconfigure::Server<ohm_rrl_motiondetection2::MotionDetectionConfig>::CallbackType f;
   f = boost::bind(&callbackConfig, _1, _2);
   srv.setCallback(f);


   MotionDetectionNode::getInstance()->run();
}





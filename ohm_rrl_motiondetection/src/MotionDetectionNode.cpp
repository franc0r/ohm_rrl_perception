#include "MotionDetectionNode.h"

//#include "SmoothRect.h"


#include <dynamic_reconfigure/server.h>

namespace enc = sensor_msgs::image_encodings;
static const char WINDOW[] = "Image window";

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
  delete _bg;
}

void MotionDetectionNode::updateConfig(void)
{
  delete _bg;

  _bg = cv::createBackgroundSubtractorMOG2(_frames, _threshold); //  new cv::BackgroundSubtractorMOG2(_frames, _threshold, true);
  ROS_INFO("Update motion configuration: frames %d, threshold %d", _frames, _threshold);
}

void MotionDetectionNode::run(void)
{
//  while(ros::ok())
    ros::spin();
}


//################# PRIVATE #######################
MotionDetectionNode::MotionDetectionNode(void)
    : _it(_nh)
, _frames(3)
, _threshold(10)
, _motionDetected(false)
, _checkMotion(false)
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
//  private_nh.param("rect_topic",          rect_topic,          std::string("notification_rect"));


//  _rect.header.frame_id    = image_topic;

  _img_sub        = _it.subscribe(image_topic,        1, &MotionDetectionNode::callbackImage, this);
  _img_pub        = _it.advertise(motion_image_topic, 1);


  const auto history_ = 2;
  const auto varThreshold_ = 15;
  const auto shadowCompensation = true;
  _bg = cv::createBackgroundSubtractorMOG2(history_, varThreshold_, shadowCompensation); // new cv::BackgroundSubtractorMOG2(_frames, _threshold, true);
}

void MotionDetectionNode::callbackImage(const sensor_msgs::ImageConstPtr& img)
{

  // do nothing if nobody subscribes to the node
   if(_img_pub.getNumSubscribers() == 0)  return;

   cv::Mat frame = this->calculateContours(img);

   cv::Mat3b lbl(frame.rows, frame.cols, cv::Vec3b(0, 0, 0));


   if (_contours.size() > 0)
   {
      _motionDetected = true;

      std::vector<int> labels;
      std::vector<cv::Point> pts;
      const int n_labels = this->calculateLabels(pts, labels);

      this->calculateLabelMarkers(lbl, pts, labels, n_labels);

      this->calculateRectangles(frame);
   }


    cv_bridge::CvImage output;
    output.header   = img->header;
    output.encoding = enc::BGR8;
    output.image    = frame;
    _img_pub.publish(output.toImageMsg());


    if (_debug) {
       cv::imshow("Labels", lbl);
       cv::waitKey(30);

       cv::imshow(WINDOW, frame);
       cv::waitKey(30);
    }
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

   cv::Mat     frame(cv_ptr->image);
   cv::Mat     back, front, blurred;

   // apply gaussian blurring to reduce noise
   const auto kernel_size = _config.bluring_factor*2 + 1;
   cv::GaussianBlur(frame, blurred, cv::Size(kernel_size,kernel_size),kernel_size,kernel_size);

   _bg->apply(blurred, front, -1); // automatic learning rate
   _bg->setShadowThreshold(200.0);
   _bg->setVarMax ( 255.0);
   _bg->setHistory(_config.nr_of_frames);
   _bg->getBackgroundImage(back);

   cv::Mat thresholdedDifference_;
//   cv::subtract(front, back, thresholdedDifference_);




   cv::Mat kernel_erode = getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

   cv::erode( front, front, kernel_erode);
   cv::dilate(front, front, kernel_erode);
//   cv::dilate(front, front, kernel_erode);

//   cv::imshow("Background", back);
//   cv::imshow("front", front);
////   cv::imshow("thresholdedDifference_", thresholdedDifference_);
//   cv::waitKey(10);

   if(_config.show_contours) {
     cv::findContours(front, _contours, _hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

     // remove contors with small size
     std::vector<std::vector<cv::Point> > filtered_contours;
     for(unsigned int i=0 ; i<_contours.size() ; ++i)
     {
       if(_contours[i].size() > _config.min_contour_size)
         filtered_contours.push_back(_contours[i]);
     }

     cv::drawContours(frame, filtered_contours, -1, cv::Scalar(0,0,255), 2);
   }
   return frame;
}

unsigned int MotionDetectionNode::calculateLabels(std::vector<cv::Point>& pts, std::vector<int>& labels)
{
    // apply clustering
    const auto th2 = _config.cluster_distance_threshold; // squared radius tolerance

    // copy all contours to pts
    for (unsigned int i = 0; i < _contours.size(); i++) {
       for (unsigned int j = 0; j < _contours[i].size(); j++) {
          pts.push_back(_contours[i][j]);
       }
    }

    const int n_labels = cv::partition(pts, labels, [th2](const cv::Point& lhs, const cv::Point& rhs) {
       return ((lhs.x - rhs.x)*(lhs.x - rhs.x) + (lhs.y - rhs.y)*(lhs.y - rhs.y)) < th2;
    });

    return n_labels;

    ROS_DEBUG_STREAM("Number of Labels: " << n_labels);

}


void MotionDetectionNode::calculateLabelMarkers(cv::Mat3b& lbl,
                                                std::vector<cv::Point>& pts,
                                                std::vector<int>& labels,
                                                unsigned int n_labels)
{

   // You can save all points in the same class in a vector (one for each class), just like findContours
   std::vector<std::vector<cv::Point>> contours(n_labels);
   for (unsigned int i=0; i<pts.size(); ++i)
      contours[labels[i]].push_back(pts[i]);

   // Build a vector of random color, one for each class (label)
   std::vector<cv::Vec3b> colors;
   for (unsigned int i=0; i<n_labels; ++i)
      colors.push_back(cv::Vec3b(rand() & 255, rand() & 255, rand() & 255));

   // Draw the labels
   for (unsigned int i=0; i<pts.size(); ++i)
      lbl(pts[i]) = colors[labels[i]];


   _contours = contours;
}


void MotionDetectionNode::calculateRectangles(cv::Mat frame)
{

   ROS_DEBUG_STREAM("Size of contour: " << _contours.size());

   std::vector<cv::Scalar> centroid_color;
   centroid_color.push_back(cv::Scalar(255,  0,  0));
   centroid_color.push_back(cv::Scalar(0,  255,  0));
   centroid_color.push_back(cv::Scalar(0,    0,255));
   centroid_color.push_back(cv::Scalar(255,  0,255));


   // calculate centroid
   for (unsigned int i = 0; i<_contours.size(); i++)
   {
      unsigned int centX = 0;
      unsigned int centY = 0;

      if(_config.show_cluster_circles)
      {
        /*
         * skip small contours
         */
        if (_contours[i].size() > _config.min_contour_size)
        {
           if(i>3) continue;

           unsigned int n = 0;
           for (unsigned int j = 0; j < _contours[i].size(); j++) {
              cv::Point p = _contours[i][j];
              centX += p.x;
              centY += p.y;
              n++;
           }
           centX = centX / n;
           centY = centY / n;

           // save to member
           _centroid.x = centX;
           _centroid.y = centY;

           std::cout << "Motion Detected" << std::endl;
           circle(frame, _centroid, 50, centroid_color[i], 2);
           cv::putText(frame, "1st Motion Detected", cv::Point(20, (i+1)*20), cv::FONT_HERSHEY_COMPLEX_SMALL,
                     0.8, cvScalar(200,200,250), 1, cv::LINE_AA);
        }
      }
   }


   if(_config.show_cluster_rects)
   {

     // draw bounding box
     for (unsigned int i = 0; i < _contours.size(); i++)
     {
        if(i>3) continue;


        int maxX = 0, minX = frame.cols;
        int maxY = 0, minY = frame.rows;


        for (unsigned int j = 0; j < _contours[i].size(); j++)
        {
           const cv::Point p = _contours[i][j];
           maxX = std::max(maxX, p.x);    maxY = std::max(maxY, p.y);
           minX = std::min(minX, p.x);    minY = std::min(minY, p.y);
        }

        if (minX > 0 && minY > 0 && maxX < frame.cols && maxY < frame.rows)
        {
           rectangle(frame, cv::Point(minX, minY), cv::Point(maxX, maxY), centroid_color[i]);

           if (maxX > minX && maxY > minY)
           {
              // smooth rect

            /*  _rect.u      = minX;
              _rect.v      = minY;
              _rect.width  = maxX - minX;
              _rect.height = maxY - minY;
              _rect.type   = "motion";

              _notify_pub.publish(smoothing::smooth(_rect)); */
           }
        }
     }
   }
}


void callbackConfig(ohm_motiondetection::MotionDetectionConfig &config, uint32_t level)
{
  ROS_INFO("Updated config via dynamic reconfigure");
  MotionDetectionNode::getInstance()->setConfig(config);
}




/*
 * Main program
 */
int main(int argc,char **argv)
{
   ros::init(argc, argv, "MotionDetection");
   ROS_INFO_STREAM("Starting motion detection node");


   // initialize dynamic reconfigure
   dynamic_reconfigure::Server<ohm_motiondetection::MotionDetectionConfig> srv;
   dynamic_reconfigure::Server<ohm_motiondetection::MotionDetectionConfig>::CallbackType f;
   f = boost::bind(&callbackConfig, _1, _2);
   srv.setCallback(f);


   MotionDetectionNode::getInstance()->run();
}





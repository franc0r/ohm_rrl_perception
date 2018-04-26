
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
//#include <QTransform>
//#include <QDebug>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
namespace enc = sensor_msgs::image_encodings;


//#include <ohm_perception_msgs/Marker.h>


#include <image_transport/image_transport.h>

image_transport::Subscriber          _img_sub;        //!< subscriber to image topic
image_transport::Publisher           _img_pub;
ros::Publisher                       _label_pub;

//todo: subscriber to image
//        publish image with marker


ros::Publisher marker_pub;
ros::Subscriber image_sub;


std::map<unsigned int, std::string> _label;

struct Hazmat
{
  std::string label;
  std::vector<cv::Point> corners;
  cv::Point one_corner;
};


std::vector<Hazmat> _detected_haz;


void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
  if(_img_pub.getNumSubscribers() == 0) return;

  _detected_haz.clear();

   if (msg.data.size())
   {
      for (unsigned int i = 0; i < msg.data.size(); i += 12)
      {
         // get data
         const int id             = (int) msg.data[i];
         const float objectWidth  = msg.data[i + 1];
         const float objectHeight = msg.data[i + 2];


         std::cout << "detected: " << _label[id] << std::endl;

         cv::Mat homography(3,3, CV_64F);

         for(unsigned int j=0; j<homography.cols*homography.rows ; ++j)
         {
           const unsigned int u = ::floor(j/3.0);
           const unsigned int v = j%3;
           homography.at<float>(j) = msg.data[i + 3+j];
//           std::cout << j << " " << homography.at<float>(/*loor(j/3.0),j%3*/ j) << std::endl; //msg.data[i + 3+j];
         }


         std::vector<cv::Point2f> obj_corners;
         obj_corners.push_back(cv::Point2f(0,0));
         obj_corners.push_back(cv::Point2f(objectWidth, 0));
         obj_corners.push_back(cv::Point2f(0, objectHeight));
         obj_corners.push_back(cv::Point2f(objectWidth, objectHeight));



//         for(unsigned int k=0 ; k<obj_corners.size() ; ++k)
//           std::cout << "k: " << obj_corners[k] << std::endl;


         std::vector<cv::Point2f> scene_corners(4);

         if(obj_corners.size() != scene_corners.size())
           ROS_ERROR("Missmatch");

         cv::perspectiveTransform( obj_corners, scene_corners, homography);

         const cv::Point p(msg.data[i + 9], msg.data[i + 10]);

         Hazmat h;
         h.label = _label[id];

//         for(unsigned int k=0 ; k<scene_corners.size() ; ++k)
//           std::cout << "k: " << scene_corners[k] << std::endl;

         h.corners.push_back(cv::Point(scene_corners[0]));
         h.corners.push_back(cv::Point(scene_corners[1]));
         h.corners.push_back(cv::Point(scene_corners[2]));
         h.corners.push_back(cv::Point(scene_corners[3]));

         h.one_corner = p;
         _detected_haz.push_back(h);


         }
      }
}

void imgCallback(const sensor_msgs::ImageConstPtr& img)
{
  // do nothing if nobody subscribes to the node
   if(_img_pub.getNumSubscribers() == 0) return;

   /*
    * convert image to opencv
    */
   cv_bridge::CvImagePtr cv_ptr;
   try {
     cv_ptr = cv_bridge::toCvCopy(img, enc::BGR8);
   }
   catch (cv_bridge::Exception& e) {
     ROS_ERROR("cv_bridge exception: %s", e.what());
   }

   cv::Mat     frame(cv_ptr->image);


//   cv::Mat H = cv::getPerspectiveTransform(frame, frame);


//   std::cout << H.cols << " " << H.rows << H.type() << std::endl;

   for(size_t i=0 ; i<_detected_haz.size() ; ++i)
   {
     for(unsigned int j=0 ; j<4 ; ++j) {
//       std::cout << "corner at:" << _detected_haz[i].corners[(j+1)%4] << std::endl;
//       cv::line(frame, _detected_haz[i].corners[j], _detected_haz[i].corners[(j+1)%4], cv::Scalar(0, 0, 255), 3);
       cv::circle(frame, _detected_haz[i].one_corner, 80, cv::Scalar(255,0,0), 3);
//       cv::putText(frame, _detected_haz[i].label, _detected_haz[i].corners[0], cv::FONT_HERSHEY_COMPLEX_SMALL,
//                 0.8, cvScalar(200,200,250), 1, CV_AA);
       cv::putText(frame, _detected_haz[i].label, cv::Point(20, (i+1)*20), cv::FONT_HERSHEY_COMPLEX_SMALL,
                 0.8, cvScalar(200,200,250), 1, CV_AA);
     }
   }



   cv_bridge::CvImage output;
   output.header   = img->header;
   output.encoding = enc::BGR8;
   output.image    = frame;



   _img_pub.publish(output.toImageMsg());
}


int main(int argc, char** argv)
{
   ros::init(argc, argv, "objects_detected");

   ros::NodeHandle nh;
   ros::NodeHandle private_nh("~");

   image_transport::ImageTransport      _it(nh);

   std::string input_topic;
   std::string output_topic;

   ros::Subscriber subs;


   private_nh.param("input_topic",   input_topic,       std::string("image_raw"));
   private_nh.param("viz_topic",     output_topic,      std::string("img_hazmats"));

   subs = nh.subscribe("objects", 1, objectsDetectedCallback);



   _img_sub        = _it.subscribe(input_topic,        1, &imgCallback);
   _img_pub        = _it.advertise(output_topic, 1);


   _label[2]   = "Non-Flammable-Gas";
   _label[4]   = "Flammable Liquid";
   _label[5]   = "Flammable Solid";
   _label[6]   = "Oxidizer";
   _label[7]   = "Radioactive II";
   _label[8]   = "Corrosive";
   _label[9]   = "Inhalation Hazard";
   _label[10]  = "Infectious Substance";
   _label[11]  = "Explosive";
   _label[12]  = "Combustible";
   _label[13]  = "Dangerous When Wet";
   _label[14]  = "Organic Peroxide";


   ros::spin();

   return 0;
}

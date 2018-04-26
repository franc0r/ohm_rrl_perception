
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <QTransform>

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
};



std::vector<Hazmat> _detected_haz;

cv::Point toCvPoint(const QPointF qt_p)
{
  cv::Point cv_p;
  cv_p.x = qt_p.x();
  cv_p.y = qt_p.y();


  return cv_p;

}

void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
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

         // Find corners Qt
         const QTransform qtHomography(msg.data[i + 3], msg.data[i + 4],
               msg.data[i + 5], msg.data[i + 6], msg.data[i + 7],
               msg.data[i + 8], msg.data[i + 9], msg.data[i + 10],
               msg.data[i + 11]);

         const QPointF qtTopLeft     = qtHomography.map(QPointF(0, 0));
         const QPointF qtTopRight    = qtHomography.map(QPointF(objectWidth, 0));
         const QPointF qtBottomLeft  = qtHomography.map(QPointF(0, objectHeight));
         const QPointF qtBottomRight = qtHomography.map(QPointF(objectWidth, objectHeight));


         Hazmat h;
         h.label = _label[id];
         h.corners.push_back(toCvPoint(qtTopLeft));
         h.corners.push_back(toCvPoint(qtTopRight));
         h.corners.push_back(toCvPoint(qtBottomRight));
         h.corners.push_back(toCvPoint(qtBottomLeft));
         _detected_haz.push_back(h);

         }
      }
}

void imgCallback(const sensor_msgs::ImageConstPtr& img)
{
  // do nothing if nobody subscribes to the node
   if(_img_pub.getNumSubscribers() == 0)
     return;

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

   for(size_t i=0 ; i<_detected_haz.size() ; ++i)
   {
     for(unsigned int j=0 ; j<4 ; ++j) {
       cv::line(frame, _detected_haz[i].corners[j], _detected_haz[i].corners[(j+1)%4], cv::Scalar(0, 0, 255), 3);
       cv::putText(frame, _detected_haz[i].label, _detected_haz[i].corners[0], cv::FONT_HERSHEY_COMPLEX_SMALL,
                 0.8, cvScalar(200,200,250), 1, CV_AA);
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

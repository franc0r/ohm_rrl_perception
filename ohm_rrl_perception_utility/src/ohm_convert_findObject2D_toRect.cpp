
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <cv_bridge/cv_bridge.h>
namespace enc = sensor_msgs::image_encodings;

#include <QtGui/QTransform>

//#include <ohm_perception_msgs/Marker.h>


#include <image_transport/image_transport.h>

image_transport::Subscriber          _img_sub;        //!< subscriber to image topic
image_transport::Publisher           _img_pub;
ros::Publisher                       _label_pub;

//todo: subscriber to image
//        publish image with marker


ros::Publisher marker_pub;
ros::Subscriber image_sub; //test


std::map<unsigned int, std::string> _label;

struct Hazmat
{
  std::string label;
  std::vector<cv::Point> corners;
  cv::Point one_corner;
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

         // Find corners Qt
         QTransform qtHomography(msg.data[i + 3], msg.data[i + 4],
               msg.data[i + 5], msg.data[i + 6], msg.data[i + 7],
               msg.data[i + 8], msg.data[i + 9], msg.data[i + 10],
               msg.data[i + 11]);

         QPointF qtTopLeft     = qtHomography.map(QPointF(0, 0));
         QPointF qtTopRight    = qtHomography.map(QPointF(objectWidth, 0));
         QPointF qtBottomLeft  = qtHomography.map(QPointF(0, objectHeight));
         QPointF qtBottomRight = qtHomography.map(
         QPointF(objectWidth, objectHeight));




         h.corners.push_back(toCvPoint(qtTopLeft));
         h.corners.push_back(toCvPoint(qtTopRight));
         h.corners.push_back(toCvPoint(qtBottomRight));
         h.corners.push_back(toCvPoint(qtBottomLeft));




//         h.corners.push_back(cv::Point(scene_corners[0]));
//         h.corners.push_back(cv::Point(scene_corners[1]));
//         h.corners.push_back(cv::Point(scene_corners[2]));
//         h.corners.push_back(cv::Point(scene_corners[3]));

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
       cv::line(frame, _detected_haz[i].corners[j], _detected_haz[i].corners[(j+1)%4], cv::Scalar(0, 0, 255), 3);
//       cv::circle(frame, _detected_haz[i].one_corner, 80, cv::Scalar(255,0,0), 3);
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
   _label[12]  = "Spontaneously Combustible";
   _label[13]  = "Dangerous When Wet";
   _label[14]  = "Organic Peroxide";
   _label[35]  = "Inhalation Hazard";
   _label[36]  = "Toxic";
//    _label[37]  = "Radioactive";
//    _label[38]  = "Corrosive";
//    _label[39]  = "Stripes";
//    _label[40]  = "Dangerous";
//   //  _label[41]  = "Explosives 1.1 1";
//   // _labelk[42]  = "Explosives 1.1A 1";
//    _label[43]  = "Flammable Gas";
// //   _label[44]  = ""
//    _label[45]  = "Non-flammable gas";
//    _label[46]  = "Oxygen";
   _label[47]  = "Combustible";
   _label[48]  = "Flammable";
//    _label[49]  = "Fuel oil";
   _label[50]  = "Gasoline";
//    _label[51]  = "Dangerous when wet";
//    _label[52]  = "Flammable solid";
//    _label[53]  = "Spontaneously combustible";
//   //  _label[54]  = "Oxidiser";
//    _label[55]  = "Organic peroxide";
//    _label[56]  = "Poison";
//   // _label[57]  = "Infectio"
   _label[58]  = "Fissile";
   _label[59]  = "Radioactive";
//    _label[60]  = "Radioactive II";
//    _label[61]  = "Radioactive III";
//    _label[62]  = "Cargo Aircraft Only";
//   //  _label[63]  = "Inhalation Hazard New";
//    _label[64]  = "Explosives-1.1-1";
//    _label[65]  = "Blasting Agents-1.5-1";
//    _label[66]  = "Oxidizer";
//update hazmats for drz 2021
  // _label[63]  = "1.5 Blasting Agents";
  // _label[64]  = "Explosives 1.1";
  // _label[65]  = "Fuel Oil";
  // _label[66]  = "Inhalation Hazard";
  // _label[67]  = "Radioactive";
  // _label[68]  = "Spontaneously Combustible";
  // _label[69]  = "Dangerous when Wet";
  // _label[70]  = "Posion";
  // _label[71]  = "Flammable Solid";
  // _label[72]  = "Oxidizer 5.1";
  // _label[73]  = "Organic Peroxide 5.2";
  // _label[74]  = "Non-Flammable Gas";
  // _label[75]  = "Flammable Gas";
  // _label[76]  = "Corrosive";
  // _label[77]  = "Oxygen";
  _label[78]  = "Fire Extingusher";
  _label[79]  = "Fire Extingusher";
  _label[80]  = "Fire Extingusher";
  _label[81]  = "Fire Extingusher";
  _label[82]  = "Fire Extingusher";
  _label[83]  = "Fire Extingusher";
  _label[84]  = "Fire Extingusher";
  _label[85]  = "Fire Extingusher";
  _label[86]  = "Fire Extingusher";
  _label[87]  = "Fire Extingusher";
  _label[88]  = "Fire Extingusher";
  _label[89]  = "Fire Extingusher";
  _label[90]  = "Fire Extingusher";
  _label[91]  = "Fire Extingusher";
  _label[92]  = "Fire Extingusher";
  _label[93]  = "Fire Extingusher";
  _label[94]  = "Fire Extingusher";
  _label[95]  = "Fire Extingusher";
  _label[96]  = "Fire Extingusher";
  _label[97]  = "Explosive";
  _label[98]  = "Explosive";
  _label[99]  = "Fire Extingusher";
  _label[100]  = "Flammable Solid";
  _label[101]  = "Dangerous when Wet";
  _label[102]  = "Spontaneously Combustible";
  _label[103]  = "Dangerous when Wet";



   ros::spin();

   return 0;
}

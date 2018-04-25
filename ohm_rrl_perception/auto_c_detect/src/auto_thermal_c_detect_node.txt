/**
 * @file   auto_thermal_c_detect_node.cpp
 * @author Johanna Gleichauf
 * @date   Stand 04.05.2017
 *
 *
 */
#include <cstdlib>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
//#include "opencv2/nonfree/nonfree.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_field_conversion.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointField.h>
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"

using namespace cv;
using namespace std;

Mat src_unten; Mat src_unten_gray;
Mat src_links; Mat src_links_gray;
Mat src_rechts; Mat src_rechts_gray;
Mat src_oben; Mat src_oben_gray;
Mat src_los; Mat src_los_gray;
Mat src_lus; Mat src_lus_gray;
Mat src_ros; Mat src_ros_gray;
Mat src_rus; Mat src_rus_gray;
Mat img; Mat img_gray;
int thresh = 100;
int max_thresh = 255;

bool u_max=false, o_max=false, r_max=false, l_max=false, ro_max=false, ru_max=false, lo_max=false, lu_max=false;
bool u_min=false, o_min=false, r_min=false, l_min=false, ro_min=false, ru_min=false, lo_min=false, lu_min=false;
double max_area=0;
double min_area=10000;


vector<vector<Point> > g_therm_detected_contours;

RNG rng(12345);

/// Function header
void thresh_callback(int, void* );

cv::Mat _Input;
cv::Mat _InputM;
cv::Mat thermo_bin;
cv::Mat _InputR;



void callThermoCam(const sensor_msgs::Image thermal_image)
{
  cv_bridge::CvImagePtr thermo_frame;
  thermo_frame = cv_bridge::toCvCopy(thermal_image, sensor_msgs::image_encodings::MONO16);
  _InputM=thermo_frame->image;

}

void callThermoCol(const sensor_msgs::Image pi_cam_image)
{
  cv_bridge::CvImagePtr frame;
  frame = cv_bridge::toCvCopy(pi_cam_image, sensor_msgs::image_encodings::BGR8); //BGR8
  _InputR = frame->image;
};

/** @function main */
int main( int argc, char** argv )
{
  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);
  sensor_msgs::ImagePtr msg;


  //Topics Seek Thermal
  ros::Subscriber theSub = nh.subscribe<sensor_msgs::Image>("/thermal_image",1,callThermoCam);
  ros::Subscriber colSub = nh.subscribe<sensor_msgs::Image>("/thermal_image_view",1,callThermoCol);


  while((_InputM.rows == 0)){
       //std::cout << "No data" << std::endl;
       ros::spinOnce();
     }
  std::cout << "Received data" << std::endl;

  /// Load source image and convert it to gray - C unten
    src_unten = imread( "./src/auto_c_detect/C_unten.jpg", 1 );

    if(! src_unten.data )                              // Check for invalid input
        {
            cout <<  "Could not open or find the image" << std::endl ;
            return -1;
        }
    /// Load source image and convert it to gray - C links
    src_links = imread( "./src/auto_c_detect/C_links.jpg", 1 );

    if(! src_links.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }
   /// Load source image and convert it to gray - C rechts
    src_rechts = imread( "./src/auto_c_detect/C_rechts.jpg", 1 );

    if(! src_rechts.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }

      /// Load source image and convert it to gray - C oben
    src_oben = imread( "./src/auto_c_detect/C_oben.jpg", 1 );

    if(! src_oben.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }

      /// Load source image and convert it to gray - C oben links schraeg
    src_los = imread( "./src/auto_c_detect/C_links_45.jpg", 1 );

    if(! src_los.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }

    /// Load source image and convert it to gray - C links unten schraeg
    src_lus = imread( "./src/auto_c_detect/C_links_unten_45.jpg", 1 );

    if(! src_lus.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }

    /// Load source image and convert it to gray - C rechts oben schraeg
    src_ros = imread( "./src/auto_c_detect/C_rechts_45.jpg", 1 );

    if(! src_ros.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }

    /// Load source image and convert it to gray - C rechts oben schraeg
    src_rus = imread( "./src/auto_c_detect/C_rechts_unten_45.jpg", 1 );

    if(! src_rus.data )                              // Check for invalid input
    {
      cout <<  "Could not open or find the image" << std::endl ;
      return -1;
    }



  //Convert _Input to gray

  ros::Rate loop_rate(30);
  while(ros::ok())
  {


//    std::cout << "Typ: " << _InputM.type() << std::endl;

  //Thermal image: all temperature values between 30 and 40 degrees are coloured black, otherwise white
    float temp;
    thermo_bin =_InputM;
    thermo_bin.convertTo(thermo_bin, CV_8U);
    //thermo_bin.CV_8U );

//    std::cout << "Rows " << _InputM.rows << std::endl;
//    std::cout << "Cols " << _InputM.cols << std::endl;
//    std::cout << "Rows thermo_bin " << thermo_bin.rows << std::endl;
//    std::cout << "Cols thermo_bin " << thermo_bin.cols << std::endl;


    for(unsigned int x=0; x<thermo_bin.rows; x++)
    {
//      std::cout << "Thermal 1 " << std::endl;
      for(unsigned int y=0; y<thermo_bin.cols;y++)

      {
//        std::cout << "Thermal 2 " << std::endl;
        unsigned short temp_pix = _InputM.at<unsigned short>(cv::Point2f(y,x));
//        std::cout << "Thermal 3 " << std::endl;
        temp = static_cast<float>((static_cast<int>(temp_pix)-1000)/10.f);
//        std::cout << "Thermal tmp " << temp << std::endl;
        if(temp>30.0 && temp<40.0)
        {
//          std::cout << "Thermal 5 " << std::endl;

          //thermo_bin.at<cv::Vec3b>(Point2f(x,y))[0] = 0;
          thermo_bin.at<uint8_t>(x, y) = 0;
        }
        else
        {
//          std::cout << "Thermal 6 " << std::endl;
          //thermo_bin.at<cv::Vec3b>(Point2f(x,y))[0] = 255;
          thermo_bin.at<uint8_t>(x, y) = 255;
//          std::cout << "Thermal 6b " << std::endl;
//          thermo_bin.at<cv::Vec3b>(Point2f(x,y))[1] = 255;
//          thermo_bin.at<cv::Vec3b>(Point2f(x,y))[2] = 255;
//          std::cout << "Thermal 6 b " << std::endl;
        }

      }
    }


//  bitwise_not(img_gray, img_gray);
//  img_gray.convertTo(img_gray, -1, 1.2 , 0); std::endl;

//    std::cout << "Thermal 9 " << std::endl;


//  cvtColor(thermo_bin, thermo_bin, CV_BGR2GRAY);
  blur(thermo_bin, thermo_bin, Size(5,5));
//  std::cout << "Thermal 10 " << std::endl;

  /// Convert image to gray and blur it
  cvtColor( src_unten, src_unten_gray, CV_BGR2GRAY );
  blur( src_unten_gray, src_unten_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_links, src_links_gray, CV_BGR2GRAY );
  blur( src_links_gray, src_links_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_rechts, src_rechts_gray, CV_BGR2GRAY );
  blur( src_rechts_gray, src_rechts_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_oben, src_oben_gray, CV_BGR2GRAY );
  blur( src_oben_gray, src_oben_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_los, src_los_gray, CV_BGR2GRAY );
  blur( src_los_gray, src_los_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_lus, src_lus_gray, CV_BGR2GRAY );
  blur( src_lus_gray, src_lus_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_ros, src_ros_gray, CV_BGR2GRAY );
  blur( src_ros_gray, src_ros_gray, Size(3,3) );
  /// Convert image to gray and blur it
  cvtColor( src_rus, src_rus_gray, CV_BGR2GRAY );
  blur( src_rus_gray, src_rus_gray, Size(3,3) );




  /// Create Window

  namedWindow( "Window",  CV_WINDOW_AUTOSIZE );
  imshow("Window", thermo_bin);



  createTrackbar( " Canny thresh:", "Window", &thresh, max_thresh, thresh_callback );
  thresh_callback( 0, 0 );


//  waitKey(0); //auskommentieren damit dauerhaft checkt
  if (waitKey(30) >= 0)
    break;
  ros::spinOnce();
  loop_rate.sleep();

  }
}

/** @function thresh_callback */
void thresh_callback(int, void* )
{
  Mat canny_output;
  Mat canny_thermal;
  Mat canny_unten_fixed;
  Mat canny_links_fixed;
  Mat canny_rechts_fixed;
  Mat canny_oben_fixed;
  Mat canny_los_fixed;
  Mat canny_lus_fixed;
  Mat canny_ros_fixed;
  Mat canny_rus_fixed;

  vector<vector<Point> > contours;
  vector<vector<Point> > contours_thermal;
  vector<vector<Point> > contours_unten_fixed;
  vector<vector<Point> > contours_links_fixed;
  vector<vector<Point> > contours_rechts_fixed;
  vector<vector<Point> > contours_oben_fixed;
  vector<vector<Point> > contours_los_fixed;
  vector<vector<Point> > contours_lus_fixed;
  vector<vector<Point> > contours_ros_fixed;
  vector<vector<Point> > contours_rus_fixed;
  vector<Vec4i> hierarchy;

  std::cout << "thermal 14 " << std::endl;
//  std::cout << "size thermo_bin " << thermo_bin.size() << std::endl;
//Reference image

//  Detect edges using canny
  Canny( src_unten_gray, canny_unten_fixed, thresh, thresh*2, 3 );
  /// Find contours
  findContours( canny_unten_fixed, contours_unten_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  Canny( src_links_gray, canny_links_fixed, thresh, thresh*2, 3 );
  //  std::cout << "before find contours " << std::endl;
    /// Find contours
  findContours( canny_links_fixed, contours_links_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

  Canny(src_rechts_gray, canny_rechts_fixed, thresh, thresh*2,3);
  findContours(canny_rechts_fixed, contours_rechts_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  Canny(src_oben_gray, canny_oben_fixed, thresh, thresh*2,3);
  findContours(canny_oben_fixed, contours_oben_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  Canny(src_los_gray, canny_los_fixed, thresh, thresh*2,3);
  findContours(canny_los_fixed, contours_los_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  Canny(src_lus_gray, canny_lus_fixed, thresh, thresh*2,3);
  findContours(canny_lus_fixed, contours_lus_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  Canny(src_ros_gray, canny_ros_fixed, thresh, thresh*2,3);
  findContours(canny_ros_fixed, contours_ros_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

  Canny(src_rus_gray, canny_rus_fixed, thresh, thresh*2,3);
  findContours(canny_rus_fixed, contours_rus_fixed, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));


  std::cout << "thermal 15 " << std::endl;

  //Thermal Image
  Canny(thermo_bin, canny_thermal, thresh, thresh*2,3);
  findContours(canny_thermal, contours_thermal, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point(0,0));

  /// Get the moments
  //Moments C unten
  vector<Moments> mu_u(contours_unten_fixed.size() );
  for( unsigned int i = 0; i < contours_unten_fixed.size(); i++ )
     { mu_u[i] = cv::moments(contours_unten_fixed[i], false);

     }

  //Moments C links
  vector<Moments> mu_l(contours_links_fixed.size() );
  for( unsigned int i = 0; i < contours_links_fixed.size(); i++ )
     { mu_l[i] = cv::moments(contours_links_fixed[i], false);

     }

  //Moments C links
  vector<Moments> mu_r(contours_rechts_fixed.size() );
  for( unsigned int i = 0; i < contours_rechts_fixed.size(); i++ )
  { mu_r[i] = cv::moments(contours_rechts_fixed[i], false);

  }

  //Moments C oben
  vector<Moments> mu_o(contours_oben_fixed.size() );
  for( unsigned int i = 0; i < contours_oben_fixed.size(); i++ )
  { mu_o[i] = cv::moments(contours_oben_fixed[i], false);

  }

  //Moments C links oben schraeg
   vector<Moments> mu_los(contours_los_fixed.size() );
   for( unsigned int i = 0; i < contours_los_fixed.size(); i++ )
   { mu_los[i] = cv::moments(contours_los_fixed[i], false);

   }

   //Moments C links unten schraeg
   vector<Moments> mu_lus(contours_lus_fixed.size() );
   for( unsigned int i = 0; i < contours_lus_fixed.size(); i++ )
   { mu_lus[i] = cv::moments(contours_lus_fixed[i], false);

   }

   //Moments C recht oben schraeg
   vector<Moments> mu_ros(contours_ros_fixed.size() );
   for( unsigned int i = 0; i < contours_ros_fixed.size(); i++ )
   { mu_ros[i] = cv::moments(contours_ros_fixed[i], false);

   }

   //Moments C recht oben schraeg
   vector<Moments> mu_rus(contours_rus_fixed.size() );
   for( unsigned int i = 0; i < contours_rus_fixed.size(); i++ )
   { mu_rus[i] = cv::moments(contours_rus_fixed[i], false);

   }


  //Moments Thermal Image
  vector<Moments> mu(contours_thermal.size()); //Needs to be changed to mu_thermal later on
  for(unsigned int i = 0; i< contours_thermal.size(); i++)
  {
    mu[i] = cv::moments(contours_thermal[i], false);
  }

  std::cout << "thermal 16 " << std::endl;

//  for(unsigned int i=0; i<contours_thermal.size();i++)
//    {
//      std::cout << "Central moment 1 camera " << abs(mu[i].nu20) << std::endl;
//      std::cout << "Central moment 2 camera " << abs(mu[i].nu11) << std::endl;
//      std::cout << "Central moment 3 camera " << abs(mu[i].nu02) << std::endl;
//      std::cout << "Central moment 4 camera " << abs(mu[i].nu30) << std::endl;
//      std::cout << "Central moment 5 camera " << abs(mu[i].nu21) << std::endl;
//      std::cout << "Central moment 6 camera " << abs(mu[i].nu12) << std::endl;
//      std::cout << "Central moment 7 camera " << abs(mu[i].nu03) << std::endl;
//
//    }


  double area_u, area_l, area_r, area_o, area_lu, area_lo, area_ru, area_ro;
  int x=0.0;
  double num[3]= {0,0,0};
  double nam[3]= {0,0,0};
  bool u_max=false, o_max=false, r_max=false, l_max=false, ro_max=false, ru_max=false, lo_max=false, lu_max=false;

  g_therm_detected_contours.clear();
  //C unten Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_unten_fixed.size(); j++)
//      if((abs(mu[i].nu20-mu_u[j].nu20)<0.009) && (abs(mu[i].nu11-mu_u[j].nu11)<0.009) && (abs(mu[i].nu02-mu_u[j].nu02)<0.009) && (abs(mu[i].nu30-mu_u[j].nu30)<0.009)&& (abs(mu[i].nu21-mu_u[j].nu21)<0.009) && (abs(mu[i].nu12-mu_u[j].nu12)<0.009) && (abs(mu[i].nu03-mu_u[j].nu03)<0.009))
      if((abs(mu[i].nu20-mu_u[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_u[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_u[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_u[j].nu30)<0.0145)&& (abs(mu[i].nu21-mu_u[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_u[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_u[j].nu03)<0.0145))

      {

        std::cout << "C unten detected " << std::endl;
        if(!u_max)
        {
        std::cout << "diff unten " << abs(mu[i].nu20-mu_u[j].nu20) << std::endl;
        area_u = contourArea(contours_thermal[i]);
        std::cout << "Area unten " << area_u << std::endl;
        num[x] = area_u;
        nam[x] = 1;

        x++;
        u_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);


       }
      }
  }


  //C links Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_links_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_l[j].nu20)<0.009) && (abs(mu[i].nu11-mu_l[j].nu11)<0.009) && (abs(mu[i].nu02-mu_l[j].nu02)<0.009) && (abs(mu[i].nu30-mu_l[j].nu30)<0.009) && (abs(mu[i].nu21-mu_l[j].nu21)<0.009) && (abs(mu[i].nu12-mu_l[j].nu12)<0.009) && (abs(mu[i].nu03-mu_l[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_l[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_l[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_l[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_l[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_l[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_l[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_l[j].nu03)<0.0145))

      {

        std::cout << "C links detected " << std::endl;
        if(!l_max)
        {
        std::cout << "diff links " << abs(mu[i].nu20-mu_l[j].nu20) << std::endl;
        area_l = contourArea(contours_thermal[i]);
        std::cout << "Area links " << area_l << std::endl;
        num[x] = area_l;
        nam[x] = 2;

        x++;
        l_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);
        }
      }
  }




  //C rechts Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_links_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_r[j].nu20)<0.009) && (abs(mu[i].nu11-mu_r[j].nu11)<0.009) && (abs(mu[i].nu02-mu_r[j].nu02)<0.009) && (abs(mu[i].nu30-mu_r[j].nu30)<0.009) && (abs(mu[i].nu21-mu_r[j].nu21)<0.009) && (abs(mu[i].nu12-mu_r[j].nu12)<0.009) && (abs(mu[i].nu03-mu_r[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_r[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_r[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_r[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_r[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_r[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_r[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_r[j].nu03)<0.0145))

      {

        std::cout << "C rechts detected " << std::endl;
        if(!r_max)
        {
        std::cout << "diff rechts " << abs(mu[i].nu20-mu_r[j].nu20) << std::endl;
        area_r = contourArea(contours_thermal[i]);
        std::cout << "Area rechts " << area_r << std::endl;
        num[x] = area_r;
        nam[x] = 3;

        x++;
        r_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);
        }
      }
  }



  //C oben Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_oben_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_o[j].nu20)<0.009) && (abs(mu[i].nu11-mu_o[j].nu11)<0.009) && (abs(mu[i].nu02-mu_o[j].nu02)<0.009) && (abs(mu[i].nu30-mu_o[j].nu30)<0.009) && (abs(mu[i].nu21-mu_o[j].nu21)<0.009) && (abs(mu[i].nu12-mu_o[j].nu12)<0.009) && (abs(mu[i].nu03-mu_o[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_o[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_o[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_o[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_o[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_o[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_o[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_o[j].nu03)<0.0145))

      {

        std::cout << "C oben detected " << std::endl;
        if(!o_max)
        {
        std::cout << "diff oben " << abs(mu[i].nu20-mu_o[j].nu20) << std::endl;
        area_o = contourArea(contours_thermal[i]);
        std::cout << "Area oben " << area_o << std::endl;
        num[x] = area_o;
        nam[x] = 4;

        x++;
        o_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);
        }
      }
  }



  //C links oben schraeg Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_los_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_los[j].nu20)<0.009) && (abs(mu[i].nu11-mu_los[j].nu11)<0.009) && (abs(mu[i].nu02-mu_los[j].nu02)<0.009) && (abs(mu[i].nu30-mu_los[j].nu30)<0.009) && (abs(mu[i].nu21-mu_los[j].nu21)<0.009) && (abs(mu[i].nu12-mu_los[j].nu12)<0.009) && (abs(mu[i].nu03-mu_los[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_los[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_los[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_los[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_los[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_los[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_los[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_los[j].nu03)<0.0145))

      {

        std::cout << "C links oben schraeg detected " << std::endl;
        if(!lo_max)
        {
        std::cout << "diff links oben schraeg " << abs(mu[i].nu20-mu_los[j].nu20) << std::endl;
        area_lo = contourArea(contours_thermal[i]);
        std::cout << "Area links oben " << area_lo << std::endl;
        num[x] = area_lo;
        nam[x] = 5;

        x++;
        lo_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);
        }
      }
  }





  //C links unten schraeg Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_lus_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_lus[j].nu20)<0.009) && (abs(mu[i].nu11-mu_lus[j].nu11)<0.009) && (abs(mu[i].nu02-mu_lus[j].nu02)<0.009) && (abs(mu[i].nu30-mu_lus[j].nu30)<0.009) && (abs(mu[i].nu21-mu_lus[j].nu21)<0.009) && (abs(mu[i].nu12-mu_lus[j].nu12)<0.009) && (abs(mu[i].nu03-mu_lus[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_lus[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_lus[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_lus[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_lus[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_lus[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_lus[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_lus[j].nu03)<0.0145))

      {

        std::cout << "C links unten schraeg detected " << std::endl;
        if(!lu_max)
        {
        std::cout << "diff links unten schraeg " << abs(mu[i].nu20-mu_lus[j].nu20) << std::endl;
        area_lu = contourArea(contours_thermal[i]);
        std::cout << "Area links unten " << area_lu << std::endl;
        num[x] = area_lu;
        nam[x] = 6;

        x++;
        lu_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);
        }
      }

  }



  //C rechts oben schraeg Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_ros_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_ros[j].nu20)<0.009) && (abs(mu[i].nu11-mu_ros[j].nu11)<0.009) && (abs(mu[i].nu02-mu_ros[j].nu02)<0.009) && (abs(mu[i].nu30-mu_ros[j].nu30)<0.009) && (abs(mu[i].nu21-mu_ros[j].nu21)<0.009) && (abs(mu[i].nu12-mu_ros[j].nu12)<0.009) && (abs(mu[i].nu03-mu_ros[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_ros[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_ros[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_ros[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_ros[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_ros[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_ros[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_ros[j].nu03)<0.0145))
      {

        std::cout << "C rechts oben schraeg detected " << std::endl;
        if(!ro_max)
        {
        std::cout << "diff rechts oben schraeg " << abs(mu[i].nu20-mu_ros[j].nu20) << std::endl;
        area_ro = contourArea(contours_thermal[i]);
        std::cout << "Area rechts oben " << area_ro << std::endl;
        num[x] = area_ro;
        nam[x] = 7;

        x++;
        ro_max = true;
        g_therm_detected_contours.push_back(contours_thermal[i]);

        }
      }
  }



  //C rechts unten schraeg Detektion

  for(unsigned int i = 0; i<contours_thermal.size();i++)
  {
    for(unsigned int j=0; j<contours_rus_fixed.size();j++)
//      if((abs(mu[i].nu20-mu_rus[j].nu20)<0.009) && (abs(mu[i].nu11-mu_rus[j].nu11)<0.009) && (abs(mu[i].nu02-mu_rus[j].nu02)<0.009) && (abs(mu[i].nu30-mu_rus[j].nu30)<0.009) && (abs(mu[i].nu21-mu_rus[j].nu21)<0.009) && (abs(mu[i].nu12-mu_rus[j].nu12)<0.009) && (abs(mu[i].nu03-mu_rus[j].nu03)<0.009))
        if((abs(mu[i].nu20-mu_rus[j].nu20)<0.0145) && (abs(mu[i].nu11-mu_rus[j].nu11)<0.0145) && (abs(mu[i].nu02-mu_rus[j].nu02)<0.0145) && (abs(mu[i].nu30-mu_rus[j].nu30)<0.0145) && (abs(mu[i].nu21-mu_rus[j].nu21)<0.0145) && (abs(mu[i].nu12-mu_rus[j].nu12)<0.0145) && (abs(mu[i].nu03-mu_rus[j].nu03)<0.0145))

      {
        std::cout << "C rechts unten schraeg detected " << std::endl;
        std::cout << "diff rechts unten schraeg " << abs(mu[i].nu20-mu_rus[j].nu20) << std::endl;
        area_ru = contourArea(contours_thermal[i]);
        std::cout << "Area oben " << area_o << std::endl;
        num[x] = area_ru;
        nam[x] = 8;
        g_therm_detected_contours.push_back(contours_thermal[i]);
      }
  }



  std::cout << " Before bubblesort " << std::endl;
    std::cout << "Num " << num[0] << " Nam " << nam[0] << std::endl;
    std::cout << "Num " << num[1] << " Nam " << nam[1] << std::endl;
    std::cout << "Num " << num[2] << " Nam " << nam[2] << std::endl;




  //  Bubblesort - find maximum area, minimum area and middle sized area
    int flag = 1;    // set flag to 1 to start first pass
    double temp;             // holding variable
    int temp_nam;

    for(unsigned int i = 1; (i <= 3) && flag; i++)
    {
      flag = 0;
      for (unsigned int j=0; j < (3 -1); j++)
      {
        if (num[j+1] > num[j])      // ascending order simply changes to <
        {
          temp = num[j];             // swap elements
          num[j] = num[j+1];
          num[j+1] = temp;
          temp_nam = nam[j];
          nam[j] = nam[j+1];
          nam[j+1] = temp_nam;
          flag = 1;               // indicates that a swap occurred.
        }
      }
    }

  //  Num[0] = maxArea, num[1] = midArea, num[2] = minArea -> nur nam[] ist interessant um herauszufinden welches C
    for(unsigned int x = 0; x<sizeof(nam);x++)
    {
     if(x==0)
     {
       std::cout << "Outer " << std::endl;
       if(nam[x]==1)
         std::cout << "C Bottom " << std::endl;
       else if(nam[x]==2)
         std::cout << "C Left " << std::endl;
       else if(nam[x]==3)
         std::cout << "C Right " << std::endl;
       else if(nam[x]==4)
         std::cout << "C Top " << std::endl;
       else if(nam[x]==5)
         std::cout << "C Left Top " << std::endl;
       else if(nam[x]==6)
         std::cout << "C Left Bottom " << std::endl;
       else if(nam[x]==7)
         std::cout << "C Right Top " << std::endl;
       else if(nam[x]==8)
         std::cout << "C Right Bottom " << std::endl;
     }


     else
      if(x==1)
      {
        std::cout << "Mid " << std::endl;
        if(nam[x]==1)
          std::cout << "C Bottom " << std::endl;
        else if(nam[x]==2)
          std::cout << "C Left " << std::endl;
        else if(nam[x]==3)
          std::cout << "C Right " << std::endl;
        else if(nam[x]==4)
          std::cout << "C Top " << std::endl;
        else if(nam[x]==5)
          std::cout << "C Left Top " << std::endl;
        else if(nam[x]==6)
          std::cout << "C Left Bottom " << std::endl;
        else if(nam[x]==7)
          std::cout << "C Right Top " << std::endl;
        else if(nam[x]==8)
          std::cout << "C Right Bottom " << std::endl;
      }
     else
       if(x==2)
       {
         std::cout << "Inner " << std::endl;
         if(nam[x]==1)
           std::cout << "C Bottom " << std::endl;
         else if(nam[x]==2)
           std::cout << "C Left " << std::endl;
         else if(nam[x]==3)
           std::cout << "C Right " << std::endl;
         else if(nam[x]==4)
           std::cout << "C Top " << std::endl;
         else if(nam[x]==5)
           std::cout << "C Left Top " << std::endl;
         else if(nam[x]==6)
           std::cout << "C Left Bottom " << std::endl;
         else if(nam[x]==7)
           std::cout << "C Right Top " << std::endl;
         else if(nam[x]==8)
           std::cout << "C Right Bottom " << std::endl;

       }



    }








  ///  Get the mass centers:
  vector<Point2f> mc( contours_thermal.size() );
  for( int i = 0; i < contours_thermal.size(); i++ )
     { mc[i] = Point2f( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 ); }

  /// Draw contours
  Mat drawing = Mat::zeros( canny_thermal.size(), CV_8UC3 );
  Scalar color = Scalar( 0, 250, 0);
  for( int i = 0; i< contours_thermal.size(); i++ )
     {

       drawContours( drawing, contours_thermal, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }

  /// Show in a window
  namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
  imshow( "Contours", drawing );

//  std::cout << "thermal " << std::endl;
  drawContours(_InputR, g_therm_detected_contours,-1, color, 3);
//  std::cout << "cols" << _InputR.cols << "rows" << _InputR.rows << std::endl;
  imshow("DetectedThermalContours", _InputR);
//  std::cout << "thermal xxx" << std::endl;

  /// Calculate the area with the moments 00 and compare with the result of the OpenCV function
//  printf("\t Info: Area and Contour Length \n");
  for( int i = 0; i< contours_thermal.size(); i++ )
     {
//       printf(" * Contour[%d] - Area (M_00) = %.2f - Area OpenCV: %.2f - Length: %.2f \n", i, mu[i].m00, contourArea(contours[i]), arcLength( contours[i], true ) );
       Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
       drawContours( drawing, contours_thermal, i, color, 2, 8, hierarchy, 0, Point() );
       circle( drawing, mc[i], 4, color, -1, 8, 0 );
     }
}

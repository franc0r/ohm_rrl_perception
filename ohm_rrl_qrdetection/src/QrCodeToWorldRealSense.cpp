/*
 * QrCodeToWorldRealSense.cpp
 *
 *  Created on: 02 October 0218
 *      Author: Johanna Gleichauf
 */

#include "QrCodeToWorldRealSense.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/transforms.h>


const double EPSILON = 0.000001;
float _minDistance = 0.5f;
std::vector<cv::Point3f> RealSensePoints_ext;
sensor_msgs::PointCloud2::ConstPtr point_cloud;



QrCodeToWorldRealSense::QrCodeToWorldRealSense()
{
	ros::NodeHandle prvNh("~");
	std::string qrTopic;
	std::string topicPointCloud;
	std::string localizedQrs;

	prvNh.param("qr_topic",     		qrTopic,    	std::string("/qr/pose"));
	prvNh.param("topic_point_cloud", topicPointCloud, 	std::string("/camera/depth_registered/points"));
	prvNh.param("tf_cam_frame", 		_tfCamFrame,	std::string("nice_rs"));
	prvNh.param("tf_map_frame", 		_tfMapFrame, 	std::string("/map"));
	prvNh.param("localized_qrs",  localizedQrs,   std::string("/qr/candidate"));

	_qrSubs     		= _nh.subscribe(qrTopic, 1, &QrCodeToWorldRealSense::qrCallBack, this);
	_realSenseSubs  	= _nh.subscribe(topicPointCloud, 1, &QrCodeToWorldRealSense::realSenseCallback, this);
	_markerPub  		= _nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);
	_qrIntersectionPub = _nh.advertise<ohm_rrl_perception_msgs::Qr>(localizedQrs, 5);
}

QrCodeToWorldRealSense::~QrCodeToWorldRealSense()
{

}



void QrCodeToWorldRealSense::qrCallBack(const ohm_rrl_perception_msgs::QrArray& qr)
{
//	  std::cout << "entered qrCallBack " << std::endl;
	  _qr_ext_array = qr;
}

void QrCodeToWorldRealSense::realSenseCallback (const sensor_msgs::PointCloud2& cloud_in)
{
	cv::Point qrCenter;
	geometry_msgs::Point p;
	if(!_qr_ext_array.qr.size())
	  return;
	if(!_listener.waitForTransform(_tfMapFrame, _tfCamFrame , ros::Time(0) ,ros::Duration(1.0))) //tfMapFrame is destination frame
	{
	  ROS_ERROR_STREAM(__PRETTY_FUNCTION__ << " Timeout waiting for transform from " << _tfCamFrame << " to " << _tfMapFrame);
	  return;
	}
	tf::StampedTransform tf;

	try
	{
	_listener.lookupTransform(_tfMapFrame, _tfCamFrame, ros::Time(0), tf);
	}
	catch(tf::TransformException& ex)
	{
	  std::cout << __PRETTY_FUNCTION__ << " error " << ex.what() << std::endl;
	  return;
	}

	for(unsigned int i=0; i < _qr_ext_array.qr.size(); i++)
	{
		p.x = 0;
		p.y = 0;
		p.z = 0;
		_qr_ext = _qr_ext_array.qr[i];
		qrCenter.x = _qr_ext.u;
		qrCenter.y = _qr_ext.v;
		pixelTo3DPoint(cloud_in, qrCenter.x, qrCenter.y, p);

		tf::Vector3 vec(p.x, p.y, p.z);
		vec = tf * vec;
		p.x = vec.x();
		p.y = vec.y();
		p.z = vec.z();
//		std::cout << "u " << qrCenter.x << " v " << qrCenter.y << std::endl;
//		std::cout << " point: " << i << " x component " << p.x << " y component " << p.y << "point z component " << p.z << std::endl;
		unsigned int id = 0;//one for each marker
		visualization_msgs::Marker marker;
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.id = id++;
		marker.type = visualization_msgs::Marker::CUBE;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime.sec = 2;
		marker.pose.position.x = p.x;
		marker.pose.position.y = p.y;
		marker.pose.position.z = p.z;
		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 0.025;
		marker.scale.y = 0.1;
		marker.scale.z = 0.05;
		marker.color.a = 0.5;
		marker.color.r = 0.0;
		marker.color.g = 1.0;
		marker.color.b = 0.0;
//		std::cout << "publish marker " << std::endl;
		_markerPub.publish(marker);

		_qr_ext.pose.position.x = p.x;
		_qr_ext.pose.position.y = p.y;
		_qr_ext.pose.position.z = p.z;
		_qrIntersectionPub.publish(_qr_ext);

	}


}


///Function to deliver 3D point out of pointcloud2
void QrCodeToWorldRealSense::pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud, const int u, const int v, geometry_msgs::Point &p)
{
//  std::cout << " enter pixel to 3D point " << std::endl;
  // get width and height of 2D point cloud data
  int width = pCloud.width;
//  std::cout << " width point cloud " << width;
  int height = pCloud.height;
//  std::cout << " height point cloud " << height;

  // Convert from u (column / width), v (row/height) to position in array
  // where X,Y,Z data starts
  // array position within RGB image
  int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

  // compute position in array where x,y,z data start
  int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
//  std::cout << " arrayPosX " << arrayPosX << " offset " << pCloud.fields[0].offset << std::endl;
  int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
//  std::cout << " arrayPosY " << arrayPosY << " offset " << pCloud.fields[1].offset << std::endl;
  int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8
//  std::cout << " arrayPosZ " << arrayPosZ << " offset " << pCloud.fields[2].offset << std::endl;

  float X = 0.0;
  float Y = 0.0;
  float Z = 0.0;

  memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
  memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
  memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

  p.x = X;
  p.y = Y;
  p.z = Z;

}


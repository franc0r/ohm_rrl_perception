/*
 * QrCodeToWorld.cpp
 *
 *  Created on: Mar 25, 2015
 *      Author: jon
 */

#include "QrCodeToWorld.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

const double EPSILON = 0.000001;
float _minDistance = 0.5f;

QrCodeToWorld::QrCodeToWorld()
{
	//TODO Camera matrix needs to be adapted to the intel realsense camera matrix
	_A <<  545.248893f, 0.0f, 305.559000f, 0.0f, 546.279802f, 250.026233f,  0.0f, 0.0f, 1.0f; // Camera matrix
	ros::NodeHandle prvNh("~");
	std::string qrTopic;
	std::string laserTopic;

	prvNh.param("qr_topic",     		qrTopic,    	std::string("/qr/pose")); ///qr/pose_left_cam
	prvNh.param("laser_topic", 			laserTopic, 	std::string("/scan")); ///georg/scan
	prvNh.param("tf_qr_frame", 			_tfQrFrame, 	std::string("/tf_qr")); ///tf_qr_left
	prvNh.param("tf_laser_frame", 		_tfLaserFrame, 	std::string("/velodyne")); ///georg/laser
	prvNh.param("tf_cam_frame", 		_tfCamFrame,	std::string("/laser")); ///georg/camera/left
	prvNh.param("tf_map_frame", 		_tfMapFrame, 	std::string("/map"));
	prvNh.param("qr_intersection_frame", 	_qrIntersectionFrame, 	std::string("/qr/candidate"));


	_qrSubs     		= _nh.subscribe(qrTopic, 1, &QrCodeToWorld::qrCallBack, this);
	_laserSubs  		= _nh.subscribe(laserTopic, 1, &QrCodeToWorld::laserCallback, this);
	_cloudPub   		= _nh.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
	_markerPub  		= _nh.advertise<visualization_msgs::Marker>("visualization_marker", 5);
	_qrIntersectionPub	= _nh.advertise<ohm_rrl_perception_msgs::Qr>(_qrIntersectionFrame, 5);
}

QrCodeToWorld::~QrCodeToWorld()
{

}

//TODO This needs to be replaced by the transformation and projection for the intel realsense camera or new node
//TODO 3D Markers and coordinates are necessary
void QrCodeToWorld::qrCallBack(const ohm_perception_msgs::QrArray& qr)
{
  std::cout << "entered qrCallBack " << std::endl;
	//i the future probably _qrArray will not be required
	cv::Point qrCenter;
	for(unsigned int i= 0; i < qr.qr.size(); i++)
	{
		_qr = qr.qr[i];
		cv::Point qrCenter;
		qrCenter.x = _qr.u;
		qrCenter.y = _qr.v;
		const Eigen::Vector3f point(qrCenter.x, qrCenter.y, 1.0f);
		const Eigen::Vector3f ray = _A.inverse() * point * 3.0;
		Eigen::Vector3f p;
		p = (Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ())).matrix() * ray;

		tf::StampedTransform transformCamera;
		ros::Time t1 = ros::Time(0);
		try{
		 // _listener.waitForTransform(_tfMapFrame, _tfCamFrame, t1, ros::Duration(4.0));
			_listener.lookupTransform( _tfMapFrame, _tfCamFrame, t1, transformCamera); /// _listener.lookupTransform(_tfMapFrame, _tfCamFrame, ros::Time(0), transformCamera)
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}

		const Eigen::Quaternionf rotation(transformCamera.getRotation().w(), transformCamera.getRotation().x(), transformCamera.getRotation().y(), transformCamera.getRotation().z());
		const Eigen::Vector3f translation(transformCamera.getOrigin().x(), transformCamera.getOrigin().y(), transformCamera.getOrigin().z());
		p = rotation * p + translation;

		unsigned int id = 0;//one for each marker
		visualization_msgs::Marker marker;

		//show an ARROW in RViz. color: green
		marker.header.frame_id = "map";
		marker.header.stamp = ros::Time();
		marker.id = id++;
		marker.type = visualization_msgs::Marker::ARROW;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime.sec = 2;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
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
		marker.points.resize(2);
		marker.points[0].x= transformCamera.getOrigin().x();
		marker.points[0].y= transformCamera.getOrigin().y();
		marker.points[0].z= 0.0;
		marker.points[1].x= p(0);
		marker.points[1].y= p(1);
		marker.points[1].z= 0.0;//p.z();
		std::cout << "publish marker " << std::endl;
		_markerPub.publish(marker);


		tf::StampedTransform transformLaser;
		ros::Time t2 = ros::Time(0);
		try{
		  //_listener.waitForTransform(_tfMapFrame, _tfLaserFrame, t2, ros::Duration(4.0));
			_listener.lookupTransform(_tfMapFrame, _tfLaserFrame, t2, transformLaser); //_listener.lookupTransform(_tfMapFrame, _tfLaserFrame, ros::Time(0), transformLaser)
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			return;
		}


		//segment intersection between the Laser_points and the QR_line
		cv::Point2f a, b, c, d, result;
		c = cv::Point2f(transformCamera.getOrigin().x(), transformCamera.getOrigin().y());
		d = cv::Point2f(p(0), p(1));
		float ma, mb, m, ta, tb, t;
		for (int i = 0; i < _laserCloud.points.size()-1; i++)
		{
			Eigen::Vector3f pL1 (_laserCloud.points[i].x,   _laserCloud.points[i].y, 1.0f);
			Eigen::Vector3f pL2 (_laserCloud.points[i+1].x, _laserCloud.points[i+1].y, 1.0f);
			const Eigen::Quaternionf rotation(transformLaser.getRotation().w(), transformLaser.getRotation().x(), transformLaser.getRotation().y(), transformLaser.getRotation().z());
			const Eigen::Vector3f translation(transformLaser.getOrigin().x(), transformLaser.getOrigin().y(), transformLaser.getOrigin().z());
			pL1 = rotation * pL1 + translation;
			pL2 = rotation * pL2 + translation;
			float distace2points = std::fabs(crossProduct(cv::Point2f(pL1(0), pL1(1)), cv::Point2f(pL2(0), pL2(1))));

			if (distace2points < 0.15)//_laserCloud.points[i].x - _laserCloud.points[i+1].x) +  std::fabs(_laserCloud.points[i].y  - _laserCloud.points[i+1].y))
			{
//								std::cout << "cro	ssProduct = " << distace2points << std::endl;
//								show the laser points in 2D in RViz. color: white
//								marker.header.frame_id = "map";
//								marker.header.stamp = ros::Time();
//								marker.id = id++;
//								marker.type = visualization_msgs::Marker::SPHERE;
//								marker.action = visualization_msgs::Marker::ADD;
//								marker.lifetime.sec = 1;
//								marker.pose.position.x = float(pL1(0));
//								marker.pose.position.y = float(pL1(1));
//								marker.scale.x = 0.02;
//								marker.scale.y = 0.02;
//								marker.scale.z = 0.02;
//								marker.color.a = 1.0;
//								marker.color.r = 0.5;
//								marker.color.g = 1.0;
//								marker.color.b = 1.0;
//								_markerPub.publish(marker);

				a = cv::Point2f(float(pL1(0)),  float(pL1(1)));
				b = cv::Point2f(float(pL2(0)),  float(pL2(1)));

				if (doLinesIntersect(a,b,c,d))
				{
					//					std::cout << "intersection successful in the laser point " << i << std::endl;
					//					std::cout << "a= " << a << "  b= " << b << "  c= " << c << "  d= " << d << std::endl;
					if (a.x == b.x)
					{
						//							std::cout << "Case A " << std::endl;
						// Case (AB)--> As a is a perfect vertical line, it cannot be represented nicely in a mathematical way.
						// we can mathematically represent line b as y = m*x + t <=> t = y - m*x
						// 										m as m = (y1-y2)/(x1-x2)
						result.x = a.x;
						m = (c.y - d.y)/ (c.x - d.x);
						t = c.y - m*c.x;
						result.y = m*result.x + t;
					}
					else if (c.x == d.x)
					{
						//							std::cout << "Case B " << std::endl;
						// Case (B)--> same but with a and b switched
						result.x = c.x;
						cv::Point2f tmpa = a;
						cv::Point2f tmpb = b;
						a=c; b=d;
						c=tmpa; d=tmpb;
						m = (c.y - d.y)/ (c.x - d.x);
						t = c.y - m*c.x;
						result.y = m*result.x + t;
					}
					else
					{
						//							std::cout << "Case C " << std::endl;
						ma = (a.y - b.y)/ (a.x - b.x);
						mb = (c.y - d.y)/ (c.x - d.x);
						ta = a.y - ma*a.x;
						tb = c.y - mb*c.x;
						result.x = (tb-ta)/(ma-mb);
						result.y= ma*result.x+ta;
					}

//					std::cout << "x= " << result.x << "  y= " << result.y <<  std::endl;

					_qr.pose.position.x = result.x;//float(pL1(0));
					_qr.pose.position.y = result.y;//float(pL1(1));
					std::cout << "publish QR intersection candidate " << std::endl;
					_qrIntersectionPub.publish(_qr);
					break;
				}
			}
		}
	}
}



//void QrCodeToWorld::laserCallBack(const sensor_msgs::LaserScan& scan)
void QrCodeToWorld::laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  sensor_msgs::LaserScan dummy = *scan_in;
  dummy.header.frame_id = "laser";
	_projector.transformLaserScanToPointCloud(_tfLaserFrame, dummy, _laserCloud, _listener);
	_cloudPub.publish(_laserCloud);
}


//(a,b)are points of a segment
double QrCodeToWorld::crossProduct(cv::Point2f a, cv::Point2f b)
{
	return a.x * b.y - b.x * a.y;
}

bool QrCodeToWorld::doBoundingBoxesIntersect(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d)
{
	if (a.x <= d.x && b.x >= c.x && a.y <= d.y	&& b.y >= c.y)
	{
		//		std::cout << __PRETTY_FUNCTION__ << std::endl;
		return true;
	}
	else
		return false;
}

//(a,b)are points of a segment. c is a point
bool QrCodeToWorld::isPointOnLine(cv::Point2f a, cv::Point2f b, cv::Point2f c)
{
//	cv::Point2f aTmp(b.x - a.x, b.y - a.y);
//	cv::Point2f bTmp(c.x - a.x, c.y - a.y);
//	double r = crossProduct(aTmp, bTmp);
//	return abs(r) < EPSILON;
}

bool QrCodeToWorld::isPointRightOfLine(cv::Point2f a, cv::Point2f b, cv::Point2f c)
{
	cv::Vec2f aTmp(b.x - a.x, b.y - a.y);
	cv::Vec2f bTmp(c.x - a.x, c.y - a.y);
	bool result = (crossProduct(aTmp, bTmp) <= 0);
	//	std::cout << "PointRightOfLine " << result << std::endl;

	return result;
}
//(a,b)are points of a segment. (c,d)are points of an other segment
bool QrCodeToWorld::lineSegmentTouchesOrCrossesLine(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d)
{
	bool result = (isPointRightOfLine(a, b, c) ^ isPointRightOfLine(a, b, d));  // || isPointOnLine(a, b, c) || isPointOnLine(a, b, d)
	return result;
}

bool QrCodeToWorld::doLinesIntersect(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d)
{
	bool result = lineSegmentTouchesOrCrossesLine(a, b, c, d) && lineSegmentTouchesOrCrossesLine(c, d, a, b);
	//	std::cout <<  a << b << c << d << std::endl;
	if (result) //doBoundingBoxesIntersect(a, b, c, d)&&
	{
		return true;
	}
	else
		return false;
}

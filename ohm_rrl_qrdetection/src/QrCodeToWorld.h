/*
 * QrCodeToWorld.h
 *
 *  Created on: Mar 25, 2015
 *      Author: jon
 */

#ifndef QRCODETOWORLD_H_
#define QRCODETOWORLD_H_

#include <ros/ros.h>
#include <opencv/cv.h>
#include <Eigen/Geometry>
#include <cmath>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose2D.h>
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <geometry_msgs/Point.h>

#include "ohm_rrl_perception_msgs/QrArray.h"
#include "ohm_rrl_perception_msgs/Qr.h"


class QrCodeToWorld {
public:
	QrCodeToWorld();
	virtual ~QrCodeToWorld();

	void qrCallBack(const ohm_rrl_perception_msgs::QrArray& qr);
	void laserCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in);
	bool doBoundingBoxesIntersect(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);
	double crossProduct(cv::Point2f a, cv::Point2f b);
	bool isPointOnLine(cv::Point2f a, cv::Point2f b, cv::Point2f c);
	bool isPointRightOfLine(cv::Point2f a, cv::Point2f b, cv::Point2f c);
	bool lineSegmentTouchesOrCrossesLine(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);
	bool doLinesIntersect(cv::Point2f a, cv::Point2f b, cv::Point2f c, cv::Point2f d);

private:
	ros::NodeHandle _nh;
	ros::Publisher  _markerPub;
	ros::Publisher  _cloudPub;
	ros::Publisher  _qrIntersectionPub;
	ros::Subscriber _qrSubs;
	ros::Subscriber _laserSubs;
	ros::Rate*      _rate;

	ohm_rrl_perception_msgs::Qr _qr;
	std::vector<ohm_rrl_perception_msgs::Qr> _intersections;
	sensor_msgs::PointCloud _laserCloud;
	laser_geometry::LaserProjection _projector;
	geometry_msgs::Point _resultP;
	tf::TransformListener _listener;
	tf::TransformListener _listenerLaser;
	Eigen::Matrix3f _A;

	std::string _tfQrFrame;
	std::string _tfLaserFrame;
	std::string _tfCamFrame;
	std::string _tfMapFrame;
	std::string _qrIntersectionFrame;
};

#endif /* QRCODETOWORLD_H_ */

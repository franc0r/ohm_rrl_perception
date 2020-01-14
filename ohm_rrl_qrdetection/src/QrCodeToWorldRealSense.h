/*
 * QrCodeToWorldRealSense.h
 *
 *  Created on: 02 October 2018
 *      Author: Johanna Gleichauf
 */

#ifndef QRCODETOWORLDREALSENSE_H_
#define QRCODETOWORLDREALSENSE_H_

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
#include "sensor_msgs/point_field_conversion.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointField.h"
#include <vector>
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "ohm_rrl_perception_msgs/QrArray.h"
#include "ohm_rrl_perception_msgs/Qr.h"


class QrCodeToWorldRealSense {
public:
	QrCodeToWorldRealSense();
	virtual ~QrCodeToWorldRealSense();

	void qrCallBack(const ohm_rrl_perception_msgs::QrArray& qr);
	void realSenseCallback (const sensor_msgs::PointCloud2& cloud_in);
	void pixelTo3DPoint(const sensor_msgs::PointCloud2& pCloud, const int u, const int v, geometry_msgs::Point &p);


private:
	ros::NodeHandle _nh;
	ros::Publisher  _markerPub;
	ros::Publisher  _qrIntersectionPub;
	ros::Subscriber _qrSubs;
	ros::Subscriber _realSenseSubs;

	std::vector<ohm_rrl_perception_msgs::Qr> _intersections;

	tf::TransformListener _listener;

	ohm_rrl_perception_msgs::Qr _qr_ext;
	ohm_rrl_perception_msgs::QrArray _qr_ext_array;

	std::string _tfCamFrame;
	std::string _tfMapFrame;

};

#endif /* QRCODETOWORLDREALSENSE_H_ */

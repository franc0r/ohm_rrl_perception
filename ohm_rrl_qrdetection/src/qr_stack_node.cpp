/*
 * ar_stack_node.cpp
 *
 *  Created on: Apr 14, 2015
 *      Author: jon
 */

#include <ros/ros.h>
#include "ohm_perception_msgs/QrArray.h"
#include "ohm_perception_msgs/Qr.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <fstream>


using namespace std;

float _minDistance = 1.0f;
visualization_msgs::MarkerArray _markers;
unsigned int _id = 0;
ros::Publisher _pubMarkers;
ros::Publisher _pubValidQrs;

ohm_perception_msgs::QrArray _qrIntersections;

std::string _qrTxtAdress;


double distanceCalculate(double x1, double y1, double x2, double y2)
{
    double x = x1 - x2;
    double y = y1 - y2;
    double dist;

    dist = pow(x,2)+pow(y,2);           //calculating distance by euclidean formula
    dist = sqrt(dist);                  //sqrt is function in math.h

    return dist;
}

void sendMarkers(void)
{
	if (!_pubMarkers.getNumSubscribers())
		return;

	//show the intersection point in RViz. color: red
	visualization_msgs::Marker marker;
	_markers.markers.clear();

	marker.header.frame_id = "map";
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.1;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	for (unsigned int i=0;i < _qrIntersections.qr.size(); i++)
	{
		marker.header.stamp = ros::Time();
		marker.id = i;
		// TODO This needs to be done in 3D
		marker.pose.position.x = _qrIntersections.qr[i].pose.position.x;
		marker.pose.position.y = _qrIntersections.qr[i].pose.position.y;
		_markers.markers.push_back(marker);
	}

	_pubMarkers.publish(_markers);
}

void writeInFile(ohm_perception_msgs::Qr msg)
{
//	ofstream myfile ("/home/jon/catkin_ws/src/ohm_perception/src/ohm_qr_detection/QrCodesLocalization.txt",  ios::ate);
//	for (unsigned int i=0;i < _qrIntersections.qr.size(); i++)
//	{
//		if (myfile.is_open())
//		{
//			myfile << _qrIntersections.qr[i]   << endl;
//			//			myfile << _qrIntersections.qr[i].pose << endl;
//		}
//		else
//			cout << "Unable to open file" << endl;
//	}
//	myfile.close();
}


void qrIntersectionCallback (const ohm_perception_msgs::Qr& msgReceived)
{
	bool newQr = true;
	ohm_perception_msgs::Qr msg = msgReceived;
	if (!_qrIntersections.qr.size())
	{
		msg.id = _qrIntersections.qr.size();
		_qrIntersections.qr.push_back(msg);
		std::cout << "we already found " << _qrIntersections.qr.size() << " Qr_codes." << std::endl;
		std::cout << "the new Qr is in: X: " << msg.pose.position.x << "  Y: " << msg.pose.position.y << std::endl;
		std::cout << "the data in it is: " << msg.data << std::endl;
//		writeInFile(msg);
	}
	else
	{
		for(unsigned int i =0; i < _qrIntersections.qr.size(); i++)
		{
			//looking if the new Qr has the same Data as the already localized Qr and cheking the localization.
			bool differentData = true;
			if(msg.data == _qrIntersections.qr[i].data)
			{
//			    cout << "We already found this QrCode Data" << endl;
				differentData = false;
			}
			double distance = distanceCalculate(msg.pose.position.x, msg.pose.position.y,
												_qrIntersections.qr[i].pose.position.x,  _qrIntersections.qr[i].pose.position.y);
			//			std::cout << "distance between two different Ar_trackers is : " << distance << std::endl;
			if (!(distance 	> _minDistance && newQr && differentData))
			{
				newQr = false;
			}
		}
		if(newQr)
		{
			msg.id = _qrIntersections.qr.size();
			_qrIntersections.qr.push_back(msg);
			std::cout << "we already fond " << _qrIntersections.qr.size() << " Qr_codes." << std::endl;
			std::cout << "the new Qr is in: X: " << msg.pose.position.x << "  Y: " << msg.pose.position.y << std::endl;
			std::cout << "the data in it is: " << msg.data << std::endl;
//			writeInFile(msg);
			_pubValidQrs.publish(msg);
		}
	}
	sendMarkers();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qr_stack");

	ros::NodeHandle prvNh("~");
	ros::NodeHandle nh;

	std::string qrCandidateTopic;
	std::string qrMarkersTopic;

	prvNh.param("qr_intersection_frame",     	qrCandidateTopic,    	std::string("/qr/candidate"));
	prvNh.param("qr_markers_topic",     		qrMarkersTopic,    	std::string("/qr/markers"));
	prvNh.param("qr_txt_adress",     			_qrTxtAdress,    	std::string("/home/jon/catkin_ws/src/ohm_perception/src/ohm_qr_detection/QrCodesLocalization.txt"));

	ros::Subscriber subQr(nh.subscribe(qrCandidateTopic, 10, qrIntersectionCallback));
	_pubMarkers = nh.advertise<visualization_msgs::MarkerArray>(qrMarkersTopic, 10);
	_pubValidQrs = nh.advertise<ohm_perception_msgs::Qr>("/qr/valid", 1);

	::sleep(2);
	ros::spin();
}

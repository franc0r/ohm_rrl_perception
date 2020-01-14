/*
 * qr_realsense_stack_node.cpp
 *
 *  Created on: Oct 10, 2018
 *      Author: Jon, Adapted for Realsense by: Johanna Gleichauf
 */

#include <ros/ros.h>
#include "ohm_rrl_perception_msgs/QrArray.h"
#include "ohm_rrl_perception_msgs/Qr.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>



#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;

float _minDistance = 0.01f;
visualization_msgs::MarkerArray _markers;
visualization_msgs::MarkerArray _texts;
unsigned int _id = 0;
ros::Publisher _pubMarkers;
ros::Publisher _pubValidQrs;
ros::Publisher _pubTexts;

ohm_rrl_perception_msgs::QrArray _qrIntersections;

std::string _qrTxtAdress;
std::string _qrFileName;



double distanceCalculate(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double x = x1 - x2;
    double y = y1 - y2;
    double z = z1 - z2;
    double dist;

    dist = pow(x,2)+pow(y,2)+pow(z,2);           //calculating distance by euclidean formula
    dist = sqrt(dist);                  //sqrt is function in math.h

    return dist;
}

void sendMarkers(void)
{
	if (!_pubMarkers.getNumSubscribers())
		return;

	//show the intersection point in RViz. color: red
	visualization_msgs::Marker marker;
	visualization_msgs::Marker text;
	_markers.markers.clear();
	_texts.markers.clear();

	marker.header.frame_id = "map";
	marker.type = visualization_msgs::Marker::CUBE;
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
	text.header.frame_id = "map";
	text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	text.action = visualization_msgs::Marker::ADD;
	text.pose.position.z = 0;
	text.pose.orientation.x = 0.0;
	text.pose.orientation.y = 0.0;
	text.pose.orientation.z = 0.0;
	text.pose.orientation.w = 1.0;
	text.scale.x = 0.1;
	text.scale.y = 0.1;
	text.scale.z = 0.1;
	text.color.a = 1.0;
	text.color.r = 1.0;
	text.color.g = 0.0;
	text.color.b = 0.0;



	for (unsigned int i=0;i < _qrIntersections.qr.size(); i++)
	{
		marker.header.stamp = ros::Time();
		marker.id = i;
		text.header.stamp = ros::Time();
		text.id = i;
		// TODO This needs to be done in 3D
		marker.pose.position.x = _qrIntersections.qr[i].pose.position.x;
		marker.pose.position.y = _qrIntersections.qr[i].pose.position.y;
		marker.pose.position.z = _qrIntersections.qr[i].pose.position.z;
		//TODO:: position text properly
		text.pose.position.x = _qrIntersections.qr[i].pose.position.x;
		text.pose.position.y = _qrIntersections.qr[i].pose.position.y + 0.5;
		text.pose.position.z = _qrIntersections.qr[i].pose.position.z;
		text.text = _qrIntersections.qr[i].data;
//		std::cout << "Text marker " << marker.text << std::endl;
		_texts.markers.push_back(text);
		_markers.markers.push_back(marker);
	}

	_pubMarkers.publish(_markers);
	_pubTexts.publish(_texts);
}

void writeInFile(ohm_rrl_perception_msgs::Qr msg)
{
	  std::stringstream ss;

	  ss << _qrTxtAdress << "/" << "WRS_" << "AutonOHM";

	  _qrFileName = ss.str() + "_qr.csv";

	  std::ofstream file;

	  file.open(_qrFileName.c_str(), std::ofstream::out | std::ofstream::app);
	  std::cout << __PRETTY_FUNCTION__ << " adding qr " << msg.data << std::endl;
	  if(!file.is_open())
	  {
	    std::cout << __PRETTY_FUNCTION__ << " error opening qrFile " << _qrFileName << "\n";

	  }
	  file << msg.data << "," << msg.pose.position.x << "," << msg.pose.position.y << ","
	       << msg.pose.position.z << std::endl;
	  file.close();
	  if(file.is_open())
	    std::cout << __PRETTY_FUNCTION__ << " warning! File " << _qrFileName.c_str() << " is still open!\n";




}


void qrIntersectionCallback (const ohm_rrl_perception_msgs::Qr& msgReceived)
{
	bool newQr = true;
	ohm_rrl_perception_msgs::Qr msg = msgReceived;
	if (!_qrIntersections.qr.size())
	{
		msg.id = _qrIntersections.qr.size();
		_qrIntersections.qr.push_back(msg);
		std::cout << "we already found " << _qrIntersections.qr.size() << " Qr_codes." << std::endl;
		std::cout << "the new Qr is in: X: " << msg.pose.position.x << "  Y: " << msg.pose.position.y << " Z: " << msg.pose.position.z << std::endl;
		std::cout << "the data in it is: " << msg.data << std::endl;
		writeInFile(msg);
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
			double distance = distanceCalculate(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z,
												_qrIntersections.qr[i].pose.position.x,  _qrIntersections.qr[i].pose.position.y, _qrIntersections.qr[i].pose.position.z);
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
			std::cout << "we already found " << _qrIntersections.qr.size() << " Qr_codes." << std::endl;
			std::cout << "the new Qr is in: X: " << msg.pose.position.x << "  Y: " << msg.pose.position.y << " Z: " << msg.pose.position.z << std::endl;
			std::cout << "the data in it is: " << msg.data << std::endl;
			writeInFile(msg);
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
	std::string qrTextsTopic;

	prvNh.param("qr_intersection_frame",     	qrCandidateTopic,   std::string("/qr/candidate"));
	prvNh.param("qr_markers_topic",     		qrMarkersTopic,    	std::string("/qr/markers"));
	prvNh.param("qr_texts_topic",     			qrTextsTopic,    	std::string("/qr/texts"));
	prvNh.param("qr_txt_adress",     			_qrTxtAdress,    	std::string("/home/jon/catkin_ws/src/ohm_perception/src/ohm_qr_detection/QrCodesLocalization.txt"));

	ros::Subscriber subQr(nh.subscribe(qrCandidateTopic, 10, qrIntersectionCallback));
	_pubMarkers = nh.advertise<visualization_msgs::MarkerArray>(qrMarkersTopic, 10);
	_pubTexts = nh.advertise<visualization_msgs::MarkerArray>(qrTextsTopic, 10);
	_pubValidQrs = nh.advertise<ohm_rrl_perception_msgs::Qr>("/qr/valid", 1);

	::sleep(2);
	ros::spin();
}

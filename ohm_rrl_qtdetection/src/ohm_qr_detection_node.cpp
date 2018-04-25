/*
 * ohm_qr_detection_node.cpp
 *
 *  Created on: Mar 25, 2015
 *      Author: jon
 */

#include <ros/ros.h>

#include "../../ohm_rrl_qtdetection/src/QrCodeDetection.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "oh_qr_detection_node");

  QrCodeDetection qrCodeDetection;

  ros::spin();
//  qrCodeDetection.run();

  return 0;
}

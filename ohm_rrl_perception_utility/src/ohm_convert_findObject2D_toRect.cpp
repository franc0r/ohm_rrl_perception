
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <opencv2/opencv.hpp>
#include <QTransform>

#include <ohm_perception_msgs/Marker.h>



ros::Publisher marker_pub;

void objectsDetectedCallback(const std_msgs::Float32MultiArray & msg)
{
   if (msg.data.size())
   {
      for (unsigned int i = 0; i < msg.data.size(); i += 12) {
         // get data
         int id = (int) msg.data[i];
         float objectWidth = msg.data[i + 1];
         float objectHeight = msg.data[i + 2];

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


         ohm_perception_msgs::Marker rect;
         rect.top_left.x     = qtTopLeft.x();       rect.top_left.y     = qtTopLeft.y();
         rect.top_right.x    = qtTopRight.x();      rect.top_right.y    = qtTopRight.y();
         rect.bottom_left.x  = qtBottomLeft.x();    rect.bottom_left.y  = qtBottomLeft.y();
         rect.bottom_right.x = qtBottomRight.x();   rect.bottom_right.y = qtBottomRight.y();

         marker_pub.publish(rect);
         }
      }
}


int main(int argc, char** argv) {
   ros::init(argc, argv, "objects_detected");

   ros::NodeHandle nh;
   ros::Subscriber subs;


   subs = nh.subscribe("objects", 1, objectsDetectedCallback);
   marker_pub =nh.advertise<ohm_perception_msgs::Marker>("augmented/hazmat", 1);
   ros::spin();

   return 0;
}

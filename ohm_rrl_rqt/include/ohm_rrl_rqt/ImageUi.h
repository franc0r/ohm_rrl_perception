/*
 * ImageUi.h
 *
 *  Created on: 25.04.2018
 *      Author: chris
 */

#ifndef OHM_RRL_PERCEPTION_OHM_RRL_RQT_SRC_IMAGEUI_H_
#define OHM_RRL_PERCEPTION_OHM_RRL_RQT_SRC_IMAGEUI_H_

#include <rqt_gui_cpp/plugin.h>
#include "/home/chris/workspace/catkin_ws/build/ohm_rrl_rqt/ui_ImageUi.h"

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Int64.h>


#include <QWidget>


namespace ohm_rqt
{

class ImageUi  : public rqt_gui_cpp::Plugin
{
  Q_OBJECT
public:
  ImageUi();
  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings);

  // Comment in to signal that the plugin has a way to configure it
  // bool hasConfiguration() const;
  // void triggerConfiguration();

private slots:
   void topicSelected(QString topic);

private:
  void updateImageTopics(void);



  void imageCallback(const sensor_msgs::ImageConstPtr& img);

  Ui::ImageUi ui_;
  QWidget* widget_;

  ros::NodeHandle                   _nh;

  image_transport::ImageTransport*  _it;

  image_transport::Subscriber       _image_sub;



  QImage _img;

  bool _display_hazmats;
  bool _display_motion;

  cv::Mat _received_image;


};
}
#endif /* OHM_RRL_PERCEPTION_OHM_RRL_RQT_SRC_IMAGEUI_H_ */

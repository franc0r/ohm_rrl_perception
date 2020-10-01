#ifndef PLUGIN_SENSOR_TEST_H_
#define PLUGIN_SENSOR_TEST_H_

#include <rqt_gui_cpp/plugin.h>
#include "ui_sensor_test.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

class PluginSensorTest : public rqt_gui_cpp::Plugin
{
  public:
  PluginSensorTest();
  virtual ~PluginSensorTest();
  virtual void initPlugin(qt_gui_cpp::PluginContext &context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);
  private:
    void callBackImg(const sensor_msgs::ImageConstPtr& img);
    void callBackTimer(const ros::TimerEvent& ev);
    std::unique_ptr<Ui::SensorTestUi> _guiUi;
    std::unique_ptr<QWidget> _widgetCentral;
    ros::NodeHandle _nh;
    image_transport::ImageTransport _it;
    image_transport::Subscriber _subsImg;
    image_transport::Publisher _pubImg;
    ros::Timer _timer;
};

#endif
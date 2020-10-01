#include "PluginSensorTest.h"
#include <pluginlib/class_list_macros.h>

PluginSensorTest::PluginSensorTest():_it(_nh)
 {}

PluginSensorTest::~PluginSensorTest() {}

void PluginSensorTest::initPlugin(qt_gui_cpp::PluginContext& context) 
{
  ros::NodeHandle prvNh("~");
  std::string topicImgOut;
  std::string topicImgIn;
  prvNh.param<std::string>("topic_img_out", topicImgOut, "sensor_test/source");
  prvNh.param<std::string>("topic_img_in", topicImgIn, "gripper/compressed");
  _widgetCentral = std::make_unique<QWidget>();
  _guiUi = std::make_unique<Ui::SensorTestUi>();
  _guiUi->setupUi(_widgetCentral.get());
  context.addWidget(_widgetCentral.get());
  _timer = _nh.createTimer(ros::Duration(1.0 / 20.0), &PluginSensorTest::callBackTimer, this);
  _subsImg = _it.subscribe(topicImgIn, 1, &PluginSensorTest::callBackImg, this);
  _pubImg = _it.advertise(topicImgOut, 1);
}

void PluginSensorTest::shutdownPlugin() 
{
  _pubImg.shutdown();
  _subsImg.shutdown();
}

void PluginSensorTest::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const {}

void PluginSensorTest::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings) {}

void PluginSensorTest::callBackImg(const sensor_msgs::ImageConstPtr& img)
{
  if(_guiUi->buttonPublish->isChecked())
    _pubImg.publish(img);
}

void PluginSensorTest::callBackTimer(const ros::TimerEvent& ev)
{
  ros::spinOnce();
} 

PLUGINLIB_EXPORT_CLASS(PluginSensorTest, rqt_gui_cpp::Plugin)
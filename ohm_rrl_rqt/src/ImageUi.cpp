/*
 Copyright 2016 Lucas Walter
 */

#include <pluginlib/class_list_macros.h>
#include <QStringList>

#include "ros/master.h"

#include "/home/chris/workspace/catkin_ws/src/ohm_rrl_perception/ohm_rrl_rqt/include/ohm_rrl_rqt/ImageUi.h"

#include <cv_bridge/cv_bridge.h>

#include <QPainter>

namespace ohm_rqt
{

ImageUi::ImageUi() :
      rqt_gui_cpp::Plugin(), widget_(nullptr), _it(nullptr)
, _display_hazmats(false)
, _display_motion(false)

{
   // Constructor is called first before initPlugin function, needless to say.

   // give QObjects reasonable names
   setObjectName("Sensor Test");

}

void ImageUi::initPlugin(qt_gui_cpp::PluginContext& context) {
   // access standalone command line arguments
   QStringList argv = context.argv();
   // create QWidget
   widget_ = new QWidget();
   // extend the widget with all attributes and children from UI file
   ui_.setupUi(widget_);
   // add widget to the user interface
   context.addWidget(widget_);

   ui_.comboBox->addItem(QString::fromStdString("hello"));

   // init ros
   _it = new image_transport::ImageTransport(_nh);

   this->updateImageTopics();



//  _image_sub = _it->subscribe("/image_raw", 1, &ImageUi::imageCallback, this);


//   _image_sub = _it->subscribe("/image_raw", 1, &ImageUi::imageCallback, this);

}

void ImageUi::shutdownPlugin() {
   // unregister all publishers here
}

void ImageUi::saveSettings(qt_gui_cpp::Settings& plugin_settings,
      qt_gui_cpp::Settings& instance_settings) const {

}

void ImageUi::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
      const qt_gui_cpp::Settings& instance_settings) {
   // v = instance_settings.value(k)
}

/*bool hasConfiguration() const
 {
 return true;
 }

 void triggerConfiguration()
 {
 // Usually used to open a dialog to offer the user a set of configuration
 }*/

// PRIVATE SLOTS
void ImageUi::topicSelected(QString topic) {
   std::cout << topic.toStdString() << std::endl;

//   _image_sub.shutdown();





}

// PRIVATE

void ImageUi::imageCallback(const sensor_msgs::ImageConstPtr& img)
{
   cv_bridge::CvImagePtr cv_ptr;
   try {
      cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
   }

   cv::Mat image(cv_ptr->image);


   _received_image = image;



   // save image in label
   QImage result = QImage((const unsigned char*) (image.data), image.cols, image.rows, QImage::Format_RGB888);
   unsigned int w = ui_._image->width();
   unsigned int h = ui_._image->height();
   ui_._image->setPixmap(QPixmap::fromImage(result).scaled(w, h, Qt::KeepAspectRatio));

}




void ImageUi::updateImageTopics(void)
{
   ui_.comboBox->clear();

   ros::master::V_TopicInfo topics;
   ros::master::getTopics(topics);
   for (const auto& t : topics) {
//     std::cout << t.name << " type: " << t.datatype << std::endl;
      if (t.datatype.find("Image") != std::string::npos) {
//        std::cout << "found image: " << std::endl;
         ui_.comboBox->addItem(QString::fromStdString(t.name));
      }
   }
}


}


PLUGINLIB_DECLARE_CLASS(ohm_rqt, ohm_rqt::ImageUi, ohm_rqt::ImageUi, rqt_gui_cpp::Plugin)

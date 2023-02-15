
#pragma onece

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

class ImageConverter
{
private:
  bool convertImage(const sensor_msgs::ImageConstPtr& input_image, cv::Mat &output_image);

public: //!!
  ImageConverter();
  ~ImageConverter();


}
// from position_estimator.cpp.        http://docs.ros.org/en/api/sensor_msgs/html/msg/Image.html
bool ::convertImage(const sensor_msgs::ImageConstPtr& input_image, 
                    cv::Mat &output_image){
  cv_bridge::CvImagePtr _cv_ptr, bgr8_cv_ptr, mono8_cv_ptr;
  try{
    // ROSからOpenCVの形式にtoCvCopy()で変換。cv_ptr->imageがcv::Matフォーマット
    // ros image msg -> cv_bridge -> cv::Mat                               	0 ~ 65535 !!
    //_cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::TYPE_16UC1);
    bgr8_cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::BGR8);
    //mono8_cv_ptr = cv_bridge::toCvCopy(input_image, sensor_msgs::image_encodings::MONO8);
    //output_image = cv_ptr->image;

    /* 画像出力
    cv::imshow("Image_window", output_image);
    cv::waitKey(0);
    cv::destroyWindow("Image_window");
    */
  }catch (cv_bridge::Exception& e){
	  ROS_ERROR("cv_bridge exception: %s", e.what());
	  return false;
  }
  return true;
}
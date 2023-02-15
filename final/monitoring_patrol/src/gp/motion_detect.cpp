
// detect whether a lying human moved
//

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
//#include "gp/image_converter.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <image_tranceport/image_transport.h> // cv::MatŒ`Ž®‰æ‘œ‚ÌPublish—p
#include <>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <>
#include <monitoring_patrol/DetectMotion.h>

class DetectMotion
{
private:
  //ImageConverter::convertImage convertImg; //!!
  ros::NodeHandle nh;
  image_transport::ImageTransport it_;
  // Sub 
  //ros::Subscriber rs_sub;
  image_transport::Subscriber rs_sub;
  // Service Server
  ros::ServiceServer md_ss;
  // member var
  sensor_msgs::ImageConstPtr raw_img;

  void realsenseCB(const sensor_msgs::ImageConstPtr& ros_img);
  bool convertImage(const sensor_msgs::ImageConstPtr& input_image,
                    cv::Mat &output_image);
  bool checkMotion(monitoring_patrol::DetectMotion::Request &req,
                   monitoring_patrol::DetectMotion::Response &res);

public:
  DetectMotion() : it_(nh) {
    rs_sub = it_.subscribe("/camera/rgb/image_raw", 1,
                        &DetectMotion::realsenseCB, this); // Sub
    md_ss = nh.advertiseService("/detect/motion", 
                              &DetectMotion::checkMotion, this); //Server
  }
  ~DtectMotion(){ /*cv::destroyWindow(OPENCV_WINDOW)*/ };
}

bool DetectMotion::convertImage(const sensor_msgs::ImageConstPtr& input_img,
                                cv::Mat &output_img);
{
  // Convert ros_img to CV format
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(input_img,
                                 sensor_msgs::image_encodings::BGR8);
    output_img = cv_ptr->image;
    // ‰æ‘œo—Í
    //cv::imshow("Image_window", output_image);
    //cv::waitKey(0);
    //cv::destroyWindow("Image_window");
  } 
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
  return true;
}

void DetectMotion::realsenseCB(const sensor_msgs::ImageConstPtr &ros_img){
  raw_img = ros_img;
  cv::Mat cv_img, curr_img, prev_img, result_img, gray ;
  sensor_msgs::ImageConstPtr curr_raw_img = raw_img;
  
  // check convert result
  bool convert_result = convertImage(curr_raw_img, cv_img)
  //cv_ptr->image;

  // to grey scale
  // create img to compare
  // calculate the diff between curr and ather one
  cv::absdiff(prev_img, current_img, result_img);
  cv::cvtColor(result_img, result_img, cv::COLOR_BGR2GRAY);
}

// detect whether the target moved or not in rep.target_time
bool DetectMotion::checkMotion(monitoring_patrol::DetectMotion::Request &req,
                              monitoring_patrol::DetectMotion::Response &res)
{
  ROS_INFO("Set target time: %.2lf ", req.target_time);

  cv::Mat cv_img, curr_img, prev_img, result_img, gray ;
  sensor_msgs::ImageConstPtr curr_raw_img = raw_img;
  // check convert result
  bool convert_result = convertImage(curr_raw_img, cv_img)
  //cv_ptr->image;

  // to grey scale
  // create img to compare
  // calculate the diff between curr and ather one
  cv::absdiff(prev_img, current_img, result_img);
  cv::cvtColor(result_img, result_img, cv::COLOR_BGR2GRAY);

  // detect motion while req.target_time
  while () {
    // extract the diff curent img and prev-one

  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "detect_motion_server")
  DetectMotion dm;


}

//--------------------
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <ros/ros.h>

cv::Mat prev_frame, current_frame, result_frame;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_frame = cv_ptr->image;
    cv::absdiff(prev_frame, current_frame, result_frame);
    cv::cvtColor(result_frame, result_frame, cv::COLOR_BGR2GRAY);
    cv::threshold(result_frame, result_frame, 50, 255, cv::THRESH_BINARY);

    // Find contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(result_frame, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw contours
    cv::drawContours(current_frame, contours, -1, cv::Scalar(0, 255, 0), 2);

    cv::imshow("Motion Detection", current_frame);
    cv::waitKey(1);

    prev_frame = current_frame.clone();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "motion_detection");
    ros::NodeHandle nh;

    cv::namedWindow("Motion Detection");
    cv::startWindowThread();

    // Subscribe to the RealSense camera topic
    ros::Subscriber sub = nh.subscribe("/camera/color/image_raw", 1, imageCallback);

    ros::spin();

    cv::destroyWindow("Motion Detection");

    return 0;
}

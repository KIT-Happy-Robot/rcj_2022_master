
// detect whether a lying human moved
//

#include <ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_tranceport/image_transport.h> //!!
#include

#include <sensor_msgs/Image.h>
#include <geometry_msgs.h>
#include <sensor_msgs/Image_encodings.h>
#include <>
#include <monitoring_patrol/DetectMotion.h>

class DetectMotion
{
public:
  DetectMotion();
  ~DtectMotion(){};

}

private:
  ros::NodeHandle nh;
  //
  ros::Subscriber rs_sub;
  ros::Subscriber ;
  ros
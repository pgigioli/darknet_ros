#ifndef ROS_INTERFACE
#define ROS_INTERFACE

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/gpu/gpu.hpp>

IplImage* get_Ipl_image();

typedef struct {
  float x, y, w, h;
  int num, Class;
} ROS_box;

#endif

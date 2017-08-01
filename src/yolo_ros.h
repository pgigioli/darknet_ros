#ifndef YOLO_ROS
#define YOLO_ROS

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

#include "cuda_runtime.h"
#include "curand.h"
#include "cublas_v2.h"

IplImage* get_Ipl_image();
/*
typedef struct {
  float x, y, w, h;
  int num, Class;
} ROS_box;
*/
typedef struct {
  float x, y, w, h, prob;
  int Class;
} PredBox;
#endif

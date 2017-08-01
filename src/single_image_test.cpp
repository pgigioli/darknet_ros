#include "yolo_ros.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <iostream>
#include <pthread.h>
#include <std_msgs/Int8.h>
#include <math.h>
#include <darknet_ros/bbox_array.h>
#include <darknet_ros/bbox.h>

extern "C" {
  #include "box.h"
}

// initialize YOLO functions that are called in this script
PredBox *run_yolo();
void load_net(char *cfgfile, char *weightfile, float thresh, float hier);
int get_obj_count();

// define demo_yolo inputs
char *cfg = "/home/catkin_ws/src/darknet_ros/cfg/tiny-yolo-voc.cfg";
char *weights = "/home/catkin_ws/src/darknet_ros/weights/tiny-yolo-voc.weights";
float thresh = 0.3;

const std::string class_labels[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
		     	             "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
		                     "potted plant", "sheep", "sofa", "train", "tv monitor" };
const int num_classes = sizeof(class_labels)/sizeof(class_labels[0]);

cv::Mat input_frame;

// define parameters
int FRAME_W;
int FRAME_H;
int FRAME_AREA;
int FRAME_COUNT = 0;

//std::vector< std::vector<ROS_box> > _class_bboxes;
std::vector<PredBox> _class_bboxes[num_classes];
int _class_obj_count[num_classes];
cv::Scalar _bbox_colors[num_classes];
darknet_ros::bbox_array _bbox_results_msg;
PredBox* _boxes;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage
IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(input_frame);
   return ROS_img;
}

void drawBBoxes(cv::Mat &input_frame, std::vector<PredBox> &class_boxes, int &class_obj_count,
		cv::Scalar &bbox_color, const std::string &class_label)
{
   //darknet_ros::bbox bbox_result;

   for (int i = 0; i < class_obj_count; i++)
   {
      int xmin = (class_boxes[i].x - class_boxes[i].w/2)*FRAME_W;
      int ymin = (class_boxes[i].y - class_boxes[i].h/2)*FRAME_H;
      int xmax = (class_boxes[i].x + class_boxes[i].w/2)*FRAME_W;
      int ymax = (class_boxes[i].y + class_boxes[i].h/2)*FRAME_H;

      // draw bounding box of first object found
      cv::Point topLeftCorner = cv::Point(xmin, ymin);
      cv::Point botRightCorner = cv::Point(xmax, ymax);
      cv::rectangle(input_frame, topLeftCorner, botRightCorner, bbox_color, 2);
      cv::putText(input_frame, class_label, cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
		  1.0, bbox_color, 2.0);
      }
   }

void get_detections(cv::Mat &full_frame)
{
   input_frame = full_frame.clone();

   // run yolo and get bounding boxes for objects
   _boxes = run_yolo();

   // get the number of bounding boxes found
   int num = get_obj_count();

   // if at least one bbox found, draw box
   if (num > 0  && num <= 100)
   {
      std::cout << "# Objects: " << num << std::endl;

      // split bounding boxes by class
      for (int i = 0; i < num; i++)
      {
         for (int j = 0; j < num_classes; j++)
         {
            if (_boxes[i].Class == j)
            {
               std::cout << _boxes[i].x << " " << _boxes[i].y << " " << _boxes[i].w << " " << _boxes[i].h << std::endl;
               std::cout << class_labels[_boxes[i].Class] << " " << _boxes[i].prob << std::endl;
               _class_bboxes[j].push_back(_boxes[i]);
               _class_obj_count[j]++;
            }
         }
      }
      for (int i = 0; i < num_classes; i++)
      {
         if (_class_obj_count[i] > 0) drawBBoxes(input_frame, _class_bboxes[i],
					         _class_obj_count[i], _bbox_colors[i], class_labels[i]);
      }
   }

   for (int i = 0; i < num_classes; i++)
   {
      _class_bboxes[i].clear();
      _class_obj_count[i] = 0;
   }

   //cv::imshow(OPENCV_WINDOW, input_frame);
   //cv::waitKey(3);
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ROS_interface");

   cv::Mat image;
   image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);

   FRAME_W = image.size().width;
   FRAME_H = image.size().height;

   load_net(cfg, weights, thresh, 0.5);
   get_detections(image);

   return 0;
}

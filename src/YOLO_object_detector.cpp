#include "ROS_interface.h"
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
extern "C" ROS_box *demo_yolo();
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

// define demo_yolo inputs
char *cfg = "/media/ubuntu/darknet/cfg/yolo-tiny.cfg";
char *weights = "/media/ubuntu/darknet/weights/yolo-tiny.weights";
float thresh = 0.3;

std::string class_labels[] = { "aeroplane", "bicycle", "bird", "boat", "bottle", "bus", "car", "cat",
		     	       "chair", "cow", "dining table", "dog", "horse", "motorbike", "person",
		               "potted plant", "sheep", "sofa", "train", "tv monitor" };
static int num_classes = sizeof(class_labels)/sizeof(class_labels[0]);
std::vector< std::vector<ROS_box> > class_bboxes(num_classes);
std::vector<int> class_obj_count(num_classes, 0);
std::vector<cv::Scalar> bbox_colors(num_classes);
darknet_ros::bbox_array bbox_results_msg;

cv::Mat cv_ptr_copy;
static ROS_box *boxes;

// define parameters
static const std::string OPENCV_WINDOW = "YOLO object detection";
int FRAME_W;
int FRAME_H;
int FRAME_AREA;
int FRAME_COUNT = 0;

// define a function that will replace CvVideoCapture.
// This function is called in yolo_kernels and allows YOLO to receive the ROS image
// message as an IplImage
IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(cv_ptr_copy);
   return ROS_img;
}

class yoloObjectDetector
{
   ros::NodeHandle nh;

   image_transport::ImageTransport it;
   image_transport::Subscriber image_sub;
   ros::Publisher found_object_pub;
   ros::Publisher bboxes_pub;

public:
   yoloObjectDetector() : it(nh)
   {
      image_sub = it.subscribe("/usb_cam/image_raw", 1,
	                       &yoloObjectDetector::callback,this);
      found_object_pub = nh.advertise<std_msgs::Int8>("found_object", 1);
      bboxes_pub = nh.advertise<darknet_ros::bbox_array>("YOLO_bboxes", 1);

      cv::namedWindow(OPENCV_WINDOW, cv::WINDOW_NORMAL);
   }

   ~yoloObjectDetector()
   {
      cv::destroyWindow(OPENCV_WINDOW);
   }

private:
   cv::Mat drawBBoxes(cv::Mat input_frame, std::vector<ROS_box> object_boxes, int num, cv::Scalar color,
		  std::string label)
   {
      darknet_ros::bbox bbox_result;

      for (int i = 0; i < num; i++) {
	 int xmin = (object_boxes[i].x - object_boxes[i].w/2)*FRAME_W;
	 int ymin = (object_boxes[i].y - object_boxes[i].h/2)*FRAME_H;
	 int xmax = (object_boxes[i].x + object_boxes[i].w/2)*FRAME_W;
	 int ymax = (object_boxes[i].y + object_boxes[i].h/2)*FRAME_H;
	 int width = object_boxes[i].w*FRAME_W;
	 int height = object_boxes[i].h*FRAME_H;

         bbox_result.Class = label;
         bbox_result.xmin = xmin;
         bbox_result.ymin = ymin;
         bbox_result.xmax = xmax;
         bbox_result.ymax = ymax;
         bbox_results_msg.bbox_array.push_back(bbox_result);

         // draw bounding box of first object found
         cv::Point topLeftCorner = cv::Point(xmin, ymin);
         cv::Point botRightCorner = cv::Point(xmax, ymax);
	 cv::rectangle(input_frame, topLeftCorner, botRightCorner, color, 2);
         cv::putText(input_frame, label, cv::Point(xmin, ymax+15), cv::FONT_HERSHEY_PLAIN,
		 1.0, color, 2.0);
      }

      return input_frame;
   }

   void runYOLO(cv::Mat full_frame)
   {
      cv::Mat input_frame = full_frame.clone();

      // run yolo and get bounding boxes for objects
      boxes = demo_yolo();

      // get the number of bounding boxes found
      int num = boxes[0].num;

      // if at least one bbox found, draw box
      if (num > 0  && num <= 100) {
	 std::cout << "# Objects: " << num << std::endl;

	 // split bounding boxes by class
         for (int i = 0; i < num; i++) {
            for (int j = 0; j < num_classes; j++) {
               if (boxes[i].Class == j) {
                  class_bboxes[j].push_back(boxes[i]);
                  class_obj_count[j]++;
               }
            }
         }

	 // send message that an object has been detected
         std_msgs::Int8 msg;
         msg.data = 1;
         found_object_pub.publish(msg);

         for (int i = 0; i < num_classes; i++) {
            if (class_obj_count[i] > 0) input_frame = drawBBoxes(input_frame, class_bboxes[i],
					      class_obj_count[i], bbox_colors[i], class_labels[i]);
         }
         bboxes_pub.publish(bbox_results_msg);
         bbox_results_msg.bbox_array.clear();
      } else {
          std_msgs::Int8 msg;
          msg.data = 0;
          found_object_pub.publish(msg);
      }

      for (int i = 0; i < num_classes; i++) {
         class_bboxes[i].clear();
         class_obj_count[i] = 0;
      }

      cv::imshow(OPENCV_WINDOW, input_frame);
      cv::waitKey(3);
   }

   void callback(const sensor_msgs::ImageConstPtr& msg)
   {
      std::cout << "usb image received" << std::endl;

      cv_bridge::CvImagePtr cv_ptr;

      try {
         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      } catch (cv_bridge::Exception& e) {
         ROS_ERROR("cv_bridge exception: %s", e.what());
	 return;
      }

      if (cv_ptr) {
         cv_ptr_copy = cv_ptr->image.clone();

	 if (FRAME_COUNT == 0) {
            runYOLO(cv_ptr->image);
         }
	 //FRAME_COUNT++;
	 if (FRAME_COUNT == 1) FRAME_COUNT = 0;
      }
      return;
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ROS_interface");

   ros::param::get("/usb_cam/image_width", FRAME_W);
   ros::param::get("/usb_cam/image_height", FRAME_H);

   int incr = floor(255/num_classes);
   for (int i = 0; i < num_classes; i++) {
      bbox_colors[i] = cv::Scalar(255 - incr*i, 0 + incr*i, 255 - incr*i);
   }

   load_network(cfg, weights, thresh);

   yoloObjectDetector yod;
   ros::spin();
   return 0;
}

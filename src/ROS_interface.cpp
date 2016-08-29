#include "ROS_interface.h"

extern "C" void demo_yolo(char *cfgfile, char *weightfile, float thresh);
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

using namespace std;
using namespace cv;
using namespace cv::gpu;

cv::Mat cv_ptr_copy;
char *cfg = "/home/ubuntu/catkin_ws/src/darknet_ros/cfg/yolo-tiny.cfg";
char *weights = "/home/ubuntu/catkin_ws/src/darknet_ros/weights/yolo-tiny.weights";
float thresh = 0.2;

IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(cv_ptr_copy);
   return ROS_img;
}

void callback(const sensor_msgs::ImageConstPtr& msg)
{
   cout << "usb image received" << endl;
   cv_bridge::CvImagePtr cv_ptr;

   try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
   } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
   }

   if (cv_ptr) {
      cv_ptr_copy = cv_ptr->image.clone();
      demo_yolo(cfg, weights, thresh);
   }
   return;
}

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ROS_interface");

   ros::NodeHandle nh;
   image_transport::ImageTransport it(nh);
   image_transport::Subscriber image_sub;

   load_network(cfg, weights, thresh);

   image_sub = it.subscribe("/usb_cam/image_raw", 1, callback);
   ros::spin();
   return 0;
}

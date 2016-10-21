#include "ROS_interface.h"

extern "C" void demo_yolo();
extern "C" void load_network(char *cfgfile, char *weightfile, float thresh);

cv::Mat cam_image_copy;
char *cfg = "/home/ubuntu/catkin_ws/src/darknet_ros/cfg/yolo-tiny.cfg";
char *weights = "/media/ubuntu/darknet/weights/yolo-tiny.weights";
float thresh = 0.2;
const std::string CAMERA_TOPIC_NAME = "/usb_cam/image_raw";

IplImage* get_Ipl_image()
{
   IplImage* ROS_img = new IplImage(cam_image_copy);
   return ROS_img;
}

class ROS_interface
{
   ros::NodeHandle _nh;
   image_transport::ImageTransport _it;
   image_transport::Subscriber _image_sub;

public:
   ROS_interface() : _it(_nh)
   {
      _image_sub = _it.subscribe(CAMERA_TOPIC_NAME, 1, &ROS_interface::cameraCallback, this);
   }

private:
   void cameraCallback(const sensor_msgs::ImageConstPtr& msg)
   {
      std::cout << "usb image received" << std::endl;
      cv_bridge::CvImagePtr cam_image;

      try
      {
         cam_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e)
      {
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }

      if (cam_image) {
         cam_image_copy = cam_image->image.clone();
         demo_yolo();
      }
      return;
   }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "ROS_interface");

   load_network(cfg, weights, thresh);

   ROS_interface ri;
   ros::spin();
   return 0;
}

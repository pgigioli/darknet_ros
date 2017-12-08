# Darknet ROS version 2
This version of darknet provides an ROS interface for running the YOLO detection as an ROS node.  The default config uses the pascal VOC detection weights but this interface can be used with any custom weights.

## To use: 

`git clone --recursive https://github.com/pgigioli/darknet_ros.git`

In yolo_ros.cpp, modify lines:

```
char *cfg = "/home/catkin_ws/src/darknet_ros/cfg/yolo-voc.cfg";
char *weights = "/home/catkin_ws/src/darknet_ros/weights/yolo-voc.weights";
```
 
with the correct path to your yolo-voc.weights and yolo-voc.cfg.

If not using the usb_cam package, also modify the lines: 

```
const std::string CAMERA_TOPIC_NAME = "/usb_cam/image_raw";
const std::string CAMERA_WIDTH_PARAM = "/usb_cam/image_width";
const std::string CAMERA_HEIGHT_PARAM = "/usb_cam/image_height";
```
 
to your camera topic name.  Compile normally with catkin_make and run with

`rosrun darknet_ros yolo_ros`

## Launchfile:

Make sure the usb_cam package is installed and use the yolo_ros.launch with:

`roslaunch darknet_ros yolo_ros.launch`

## Topics:

/found_object - displays "1" or "0" corresponding to whether or not an object has been detected

/YOLO_bboxes  - displays the class label that was detected followed by the bbox coordinates [xmin, ymin, xmax, ymax].

## Dockerfile
Avoid incompatibility issues with this dockerfile that will work out of the box. Docker image includes ubuntu 16.04, ROS kinetic, CUDA 8 and cudnn 6.  Docker image will install darknet_ros and the usb_cam package.
 
Install docker here: https://docs.docker.com/engine/installation/linux/docker-ce/ubuntu/

Install nvidia-docker here: https://github.com/NVIDIA/nvidia-docker

Build docker image:

`docker build -t darknet_ros:latest .`

Run docker container and allow docker access to webcam:

`xhost +`

`nvidia-docker run -it --privileged -v /tmp/.X11-unix:/tmp/.X11-unix -e DISPLAY <image>`

Build darknet_ros nodes (you may have to run this twice):

`catkin_make`

`source devel/setup.bash`

Launch darknet_ros with usb_cam:

`roslaunch darknet_ros yolo_ros.launch`

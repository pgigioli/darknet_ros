# Darknet ROS version 2
This version of darknet provides an ROS interface for running the YOLO detection as an ROS node.  The default config uses the pascal VOC detection weights but this interface can be used with any custom weights.

To use: Modify yolo_ros.cpp with the correct path to your yolo-voc.weights and yolo-voc.cfg and change the /usb_cam/image_raw topic to your camera topic.  Compile normally with catkin_make and run with "rosrun darknet_ros yolo_ros".

Topics:

/found_object - displays "1" or "0" corresponding to whether or not an object has been detected

/YOLO_bboxes  - displays the class label that was detected followed by the bbox coordinates [xmin, ymin, xmax, ymax].

# Dockerfile
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

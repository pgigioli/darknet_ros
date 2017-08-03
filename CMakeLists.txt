cmake_minimum_required(VERSION 2.8.12)
project(darknet_ros)

find_package(CUDA REQUIRED)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  roscpp
  rospy
  std_msgs
  image_transport
  message_generation
)

add_message_files(
  FILES
  bbox.msg
  bbox_array.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_20,code=[sm_20,sm_21]
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_53,code=sm_53
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
    )

add_definitions(-DGPU -DOPENCV -O4 -g)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set(DARKNET_DIR "darknet")
set(DARKNET_SRC_DIR "${DARKNET_DIR}/src")

include_directories(
  /usr/local/cuda/include
  src
  ${DARKNET_DIR}/include
  ${DARKNET_SRC_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  /usr/local/cuda/lib64
)

set(DARKNET_SRC_FILES
	${DARKNET_SRC_DIR}/activation_layer.c
	${DARKNET_SRC_DIR}/activations.c
	${DARKNET_SRC_DIR}/avgpool_layer.c
	${DARKNET_SRC_DIR}/batchnorm_layer.c
	${DARKNET_SRC_DIR}/blas.c
	${DARKNET_SRC_DIR}/box.c
	${DARKNET_SRC_DIR}/col2im.c
	${DARKNET_SRC_DIR}/compare.c
	${DARKNET_SRC_DIR}/connected_layer.c
	${DARKNET_SRC_DIR}/convolutional_layer.c
	${DARKNET_SRC_DIR}/cost_layer.c
	${DARKNET_SRC_DIR}/crnn_layer.c
	${DARKNET_SRC_DIR}/crop_layer.c
	${DARKNET_SRC_DIR}/cuda.c
	${DARKNET_SRC_DIR}/data.c
	${DARKNET_SRC_DIR}/deconvolutional_layer.c
	#${DARKNET_SRC_DIR}/demo.c
	${DARKNET_SRC_DIR}/detection_layer.c
	${DARKNET_SRC_DIR}/dropout_layer.c
	${DARKNET_SRC_DIR}/gemm.c
	${DARKNET_SRC_DIR}/gru_layer.c
	${DARKNET_SRC_DIR}/im2col.c
	${DARKNET_SRC_DIR}/image.c
	${DARKNET_SRC_DIR}/layer.c
	${DARKNET_SRC_DIR}/list.c
	${DARKNET_SRC_DIR}/local_layer.c
	${DARKNET_SRC_DIR}/lstm_layer.c
	${DARKNET_SRC_DIR}/matrix.c
	${DARKNET_SRC_DIR}/maxpool_layer.c
	${DARKNET_SRC_DIR}/network.c
	${DARKNET_SRC_DIR}/normalization_layer.c
	${DARKNET_SRC_DIR}/option_list.c
	${DARKNET_SRC_DIR}/parser.c
	${DARKNET_SRC_DIR}/region_layer.c
	${DARKNET_SRC_DIR}/reorg_layer.c
	${DARKNET_SRC_DIR}/rnn_layer.c
	${DARKNET_SRC_DIR}/route_layer.c
	${DARKNET_SRC_DIR}/shortcut_layer.c
	${DARKNET_SRC_DIR}/softmax_layer.c
	${DARKNET_SRC_DIR}/tree.c
	${DARKNET_SRC_DIR}/utils.c

	${DARKNET_SRC_DIR}/activation_kernels.cu
	${DARKNET_SRC_DIR}/avgpool_layer_kernels.cu
	${DARKNET_SRC_DIR}/blas_kernels.cu
	${DARKNET_SRC_DIR}/col2im_kernels.cu
	${DARKNET_SRC_DIR}/convolutional_kernels.cu
	${DARKNET_SRC_DIR}/crop_layer_kernels.cu
	${DARKNET_SRC_DIR}/deconvolutional_kernels.cu
	${DARKNET_SRC_DIR}/dropout_layer_kernels.cu
	${DARKNET_SRC_DIR}/im2col_kernels.cu
	${DARKNET_SRC_DIR}/maxpool_layer_kernels.cu
	${DARKNET_SRC_DIR}/network_kernels.cu
)


cuda_add_executable(single_image_test
  src/single_image_test.cpp
  src/run_yolo.cpp
  "${DARKNET_SRC_FILES}"
)

cuda_add_executable(yolo_ros
  src/yolo_ros.cpp
  src/run_yolo.cpp
  "${DARKNET_SRC_FILES}"
)

target_link_libraries(single_image_test
   m
   pthread
   stdc++
   cuda
   cudart
   cublas
   curand
   ${catkin_LIBRARIES}
)

target_link_libraries(yolo_ros
   m
   pthread
   stdc++
   cuda
   cudart
   cublas
   curand
   ${catkin_LIBRARIES}
)

add_dependencies(single_image_test my_package_generate_messages_cpp)
add_dependencies(yolo_ros my_package_generate_messages_cpp)

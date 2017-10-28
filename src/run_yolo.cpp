//#ifdef GPU
//#include "cuda_runtime.h"
//#include "curand.h"
//#include "cublas_v2.h"
//#endif

#include "yolo_ros.h"
#include <iostream>

extern "C"{
#include "network.h"
#include "detection_layer.h"
#include "region_layer.h"
#include "cost_layer.h"
#include "utils.h"
#include "parser.h"
#include "box.h"
#include "image.h"
#include "demo.h"
#include <sys/time.h>
}

#ifdef OPENCV
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

extern "C" image ipl_to_image(IplImage* src);

//static char **demo_names;
//static image **demo_alphabet;
//static int demo_classes;

static float **probs;
static box *boxes;
static network net;
static image in;
static image in_s;
static image det_s;
static float fps = 0;
static float demo_thresh = 0;
static float demo_hier = .5;
//static int running = 0;

static int demo_frame = 1;
static int demo_detections = 0;
static float *predictions[1];
static int demo_index = 0;
//static int demo_done = 0;
static float *avg;
double demo_time;
static int obj_count = 0;

//static ROS_box *ROI_boxes;
static PredBox *pred_boxes;

double get_wall_time()
{
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void detect_objects()
{
   // running = 1;
    float nms = .4;

    layer l = net.layers[net.n-1];
    //float *X = buff_letter[(buff_index+2)%3].data;
    float *X = det_s.data;
    float *prediction = network_predict(net, X);
    free(det_s.data);

    memcpy(predictions[demo_index], prediction, l.outputs*sizeof(float));
    mean_arrays(predictions, demo_frame, l.outputs, avg);
    l.output = avg;
    if(l.type == DETECTION){
        get_detection_boxes(l, 1, 1, demo_thresh, probs, boxes, 0);
    } else if (l.type == REGION){
        get_region_boxes(l, in.w, in.h, net.w, net.h, demo_thresh, probs, boxes, 0, 0, 0, demo_hier, 1);
    } else {
        error("Last layer must produce detections\n");
    }
    if (nms > 0) do_nms_obj(boxes, probs, l.w*l.h*l.n, l.classes, nms);

    //printf("\033[2J");
    //printf("\033[1;1H");
    printf("\nFPS:%.1f\n",fps);
    //printf("Objects:\n\n");
    //image display = buff[(buff_index+2) % 3];
    //draw_detections(display, demo_detections, demo_thresh, boxes, probs, 0, demo_names, demo_alphabet, demo_classes);

    //demo_index = (demo_index + 1)%demo_frame;
    //running = 0;
   // extract the bounding boxes and send them to ROS
   int total = l.w*l.h*l.n;
   int i, j;
   obj_count = 0;
   for(i = 0; i < total; ++i)
   {
      float xmin = boxes[i].x - boxes[i].w/2.;
      float xmax = boxes[i].x + boxes[i].w/2.;
      float ymin = boxes[i].y - boxes[i].h/2.;
      float ymax = boxes[i].y + boxes[i].h/2.;

      if (xmin < 0) xmin = 0;
      if (ymin < 0) ymin = 0;
      if (xmax > 1) xmax = 1;
      if (ymax > 1) ymax = 1;

      // iterate through possible boxes and collect the bounding boxes
      for(j = 0; j < l.classes; ++j)
      {
         if (probs[i][j])
         {
            float x_center = (xmin + xmax)/2;
            float y_center = (ymin + ymax)/2;
            float bbox_width = xmax - xmin;
            float bbox_height = ymax - ymin;

            // define bounding box
            // bbox must be 1% size of frame (3.2x2.4 pixels)
            if (bbox_width > 0.01 && bbox_height > 0.01)
            {
               pred_boxes[obj_count].x = x_center;
               pred_boxes[obj_count].y = y_center;
               pred_boxes[obj_count].w = bbox_width;
               pred_boxes[obj_count].h = bbox_height;
               pred_boxes[obj_count].Class = j;
               pred_boxes[obj_count].prob = probs[i][j];
	       obj_count++;
            }
         }
      }
   }

   // create array to store found bounding boxes
   // if no object detected, make sure that ROS knows that num = 0
   /*if (count == 0)
   {
      pred_boxes[0].num = 0;
   }
   else
   {
      pred_boxes[0].num = count;
   }*/
    return;
}

int get_obj_count()
{
   return obj_count;
}

void fetch_image()
{
   IplImage* ROS_img = get_Ipl_image();
   in = ipl_to_image(ROS_img);
   delete ROS_img;
   ROS_img = NULL;
   in_s = letterbox_image(in, net.w, net.h);
   free(in.data);
   return;
}
/*
void *display_in_thread(void *ptr)
{
    show_image_cv(buff[(buff_index + 1)%3], "Demo", ipl);
    int c = cvWaitKey(1);
    if (c != -1) c = c%256;
    if (c == 27) {
        demo_done = 1;
        return 0;
    } else if (c == 82) {
        demo_thresh += .02;
    } else if (c == 84) {
        demo_thresh -= .02;
        if(demo_thresh <= .02) demo_thresh = .02;
    } else if (c == 83) {
        demo_hier += .02;
    } else if (c == 81) {
        demo_hier -= .02;
        if(demo_hier <= .0) demo_hier = .0;
    }
    return 0;
}

void *display_loop(void *ptr)
{
    while(1){
        display_in_thread(0);
    }
}

void *detect_loop(void *ptr)
{
    while(1){
        detect_in_thread(0);
    }
}*/

//void load_network(char *cfgfile, char *weightfile, float thresh, int cam_index, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
void load_net(char *cfgfile, char *weightfile, float thresh, float hier)
{
   //demo_frame = avg_frames;
   //predictions = calloc(demo_frame, sizeof(float*));
   //image **alphabet = load_alphabet();
   //demo_names = names;
   //demo_alphabet = alphabet;
   //demo_classes = classes;
   demo_thresh = thresh;
   demo_hier = hier;
   std::cout << "Loading network...\n" << std::endl;
   net = parse_network_cfg(cfgfile);
   if(weightfile)
   {
      load_weights(&net, weightfile);
   }
   set_batch_network(&net, 1);

   srand(2222222);

   layer l = net.layers[net.n-1];
   demo_detections = l.n*l.w*l.h;
   int j;

   avg = (float *) calloc(l.outputs, sizeof(float));

   for(j = 0; j < demo_frame; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));
   boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
   pred_boxes = (PredBox *)calloc(l.w*l.h*l.n, sizeof(PredBox));
   probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
   for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes+1, sizeof(float));
}

PredBox *run_yolo()
{
   fetch_image();
   det_s = in_s;
   detect_objects();

   fps = 1./(get_wall_time() - demo_time + 0.00000001);
   std::cout<<"FPS: "<<fps<<std::endl;
   demo_time = get_wall_time();
   return pred_boxes;
/*   buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]);
    buff_letter[0] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[1] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[2] = letterbox_image(buff[0], net.w, net.h);
    ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

    int count = 0;
    if(!prefix){
        cvNamedWindow("Demo", CV_WINDOW_NORMAL); 
        if(fullscreen){
            cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        } else {
            cvMoveWindow("Demo", 0, 0);
            cvResizeWindow("Demo", 1352, 1013);
        }
    }

    demo_time = get_wall_time();

    while(!demo_done){
        buff_index = (buff_index + 1) %3;
        if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
        if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
        if(!prefix){
            fps = 1./(get_wall_time() - demo_time);
            demo_time = get_wall_time();
            display_in_thread(0);
        }else{
            char name[256];
            sprintf(name, "%s_%08d", prefix, count);
            save_image(buff[(buff_index + 1)%3], name);
        }
        pthread_join(fetch_thread, 0);
        pthread_join(detect_thread, 0);
        ++count;
    }
*/
}
/*
void demo_compare(char *cfg1, char *weight1, char *cfg2, char *weight2, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg_frames, float hier, int w, int h, int frames, int fullscreen)
{
    demo_frame = avg_frames;
    predictions = calloc(demo_frame, sizeof(float*));
    image **alphabet = load_alphabet();
    demo_names = names;
    demo_alphabet = alphabet;
    demo_classes = classes;
    demo_thresh = thresh;
    demo_hier = hier;
    printf("Demo\n");
    net = load_network(cfg1, weight1, 0);
    set_batch_network(&net, 1);
    pthread_t detect_thread;
    pthread_t fetch_thread;

    srand(2222222);

    if(filename){
        printf("video file: %s\n", filename);
        cap = cvCaptureFromFile(filename);
    }else{
        cap = cvCaptureFromCAM(cam_index);

        if(w){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_WIDTH, w);
        }
        if(h){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FRAME_HEIGHT, h);
        }
        if(frames){
            cvSetCaptureProperty(cap, CV_CAP_PROP_FPS, frames);
        }
    }

    if(!cap) error("Couldn't connect to webcam.\n");

    layer l = net.layers[net.n-1];
    demo_detections = l.n*l.w*l.h;
    int j;

    avg = (float *) calloc(l.outputs, sizeof(float));
    for(j = 0; j < demo_frame; ++j) predictions[j] = (float *) calloc(l.outputs, sizeof(float));

    boxes = (box *)calloc(l.w*l.h*l.n, sizeof(box));
    probs = (float **)calloc(l.w*l.h*l.n, sizeof(float *));
    for(j = 0; j < l.w*l.h*l.n; ++j) probs[j] = (float *)calloc(l.classes+1, sizeof(float));

    buff[0] = get_image_from_stream(cap);
    buff[1] = copy_image(buff[0]);
    buff[2] = copy_image(buff[0]);
    buff_letter[0] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[1] = letterbox_image(buff[0], net.w, net.h);
    buff_letter[2] = letterbox_image(buff[0], net.w, net.h);
    ipl = cvCreateImage(cvSize(buff[0].w,buff[0].h), IPL_DEPTH_8U, buff[0].c);

    int count = 0;
    if(!prefix){
        cvNamedWindow("Demo", CV_WINDOW_NORMAL); 
        if(fullscreen){
            cvSetWindowProperty("Demo", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        } else {
            cvMoveWindow("Demo", 0, 0);
            cvResizeWindow("Demo", 1352, 1013);
        }
    }

    demo_time = get_wall_time();

    while(!demo_done){
        buff_index = (buff_index + 1) %3;
        if(pthread_create(&fetch_thread, 0, fetch_in_thread, 0)) error("Thread creation failed");
        if(pthread_create(&detect_thread, 0, detect_in_thread, 0)) error("Thread creation failed");
        if(!prefix){
            fps = 1./(get_wall_time() - demo_time);
            demo_time = get_wall_time();
            display_in_thread(0);
        }else{
            char name[256];
            sprintf(name, "%s_%08d", prefix, count);
            save_image(buff[(buff_index + 1)%3], name);
        }
        pthread_join(fetch_thread, 0);
        pthread_join(detect_thread, 0);
        ++count;
    }
}*/
#else
void demo(char *cfgfile, char *weightfile, float thresh, int cam_index, const char *filename, char **names, int classes, int delay, char *prefix, int avg, float hier, int w, int h, int frames, int fullscreen)
{
    fprintf(stderr, "Demo needs OpenCV for webcam images.\n");
}
#endif


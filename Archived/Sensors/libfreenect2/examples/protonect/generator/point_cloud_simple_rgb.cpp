/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2011 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


#include <iostream>
#include <fstream>
#include <signal.h>
#include <stdio.h>
#include <typeinfo>
#include <cxxabi.h>

#include <libfreenect2/opengl.h>

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>

#include <math.h>
#define PI 3.14159265

using namespace cv;
using namespace std;

libfreenect2::Frame *depth;
libfreenect2::Frame *rgb;

bool protonect_shutdown = false;

float depth_fov_horiz = 70.6;
float depth_fov_vert = 60;
float depth_res_horiz = 512;
float depth_res_vert = 424;
float depth_focus_to_screen = 364; // THIS CONSTANT WILL CHANGE IF ABOVE FOUR PARAMETERS CHANGE

void get_point_coord(int x_on_screen, int y_on_screen, float depth, float *x_real, float *y_real);

void sigint_handler(int s)
{
  protonect_shutdown = true;
}

// Hard-coded homography matrix. Calculated by calibrate.py in homography_calibration using corresponding_points.txt

float H11 = 2.46445726e+00;
float H12 = 2.27221406e-01;
float H13 = 2.37377866e+02;
float H21 = -1.93647364e-01;
float H22 = 2.64880019e+00;
float H23 = -5.18088189e+00;
float H31 = -3.37587064e-04;
float H32 = -5.42170683e-05;
float H33 = 1.00000000e+00;


// Calculate Hx where x is [depthx; depthy; 1]. 
void rgb_point_from_depth(int depthx, int depthy, int *rgbx, int *rgby) {
    int x = depthx;
    int y = depthy;
    int z = 1;
    float newx = H11*x+H12*y+H13*z;
    float newy = H21*x+H22*y+H23*z;
    float newz = H31*x+H32*y+H33*z;
    int newx_norm = (int)(newx/newz);
    int newy_norm = (int)(newy/newz);
    *rgbx = newx_norm;
    *rgby = newy_norm;
}

void mouse_click_event(int event, int x, int y, int flags, void *userdata) {
  if(event==EVENT_LBUTTONDOWN) {
    Mat cp = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    Mat cp_rgb = cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    float total = 0;
    float count = 0;
    int idx = (depth->width*y+x)*depth->bytes_per_pixel;
    float z_real = *((float*)(depth->data+idx));
    
    std::cout << depth->bytes_per_pixel << "\n";
    std::cout << "Left button clicked at x: " << x << " y: " << y << "\n" << "Depth value is: ";
    printf("%f\n",z_real);
    float *x_real = new float;
    float *y_real = new float;
    get_point_coord(x, y, z_real, x_real, y_real);
    float x_meter = (*x_real)/1000.0;
    float y_meter = (*y_real)/1000.0;
    float z_meter = z_real/1000.0;
    printf("The real world coordinate is: (%f, %f, %f)\n", x_meter, y_meter, z_meter);
    int *rgbx = new int;
    int *rgby = new int;
    rgb_point_from_depth(x, y, rgbx, rgby);
    Point3_<uchar>* p = cp_rgb.ptr<Point3_<uchar> >(*rgby,*rgbx);
    uint8_t b = p->x; //B
    uint8_t g = p->y; //G
    uint8_t r = p->z; //R
    printf("RGB value is %d, %d, %d\n", r, g, b);
  }
}

void rgb_mouse_click_event(int event, int x, int y, int flags, void *userdata) {
  if(event==EVENT_LBUTTONDOWN) {
    Mat cp_rgb = cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    Point3_<uchar>* p = cp_rgb.ptr<Point3_<uchar> >(y,x);
    uint8_t b = p->x; //B
    uint8_t g = p->y; //G
    uint8_t r = p->z; //R
    printf("Mouse clicked at %d, %d\n", x, y);
    printf("RGB value is %d, %d, %d\n", r, g, b);
    
  }
}

void make_point_cloud_file(char *file_name, char *img_name) {
    std::cout << "Writing to file: " << file_name << "\n";
    Mat cp_depth = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    Mat cp_rgb = cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    cv::imwrite(img_name, cp_rgb)
    int count = 0;
    //ofstream f;
    //f.open("/home/motion/libfreenect2/examples/protonect/point_cloud_data/tmp");
    for(int i=0;i<depth_res_horiz;i++) {
        for(int j=0;j<depth_res_vert;j++) {
            float z_real = cp_depth.at<float>(j,i);
            if(z_real!=0) count++;
        }
    }
    //f.close();
    ofstream o;
    o.open(file_name);
    o << "# .PCD v.7 - Point Cloud Data file format\n" << "VERSION .7\nFIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F F\n";
    o << "COUNT 1 1 1 1\nWIDTH " << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << count << "\nDATA ascii\n";
    for(int i=0;i<depth_res_horiz;i++) {
        for(int j=0;j<depth_res_vert;j++) {
            float z_real = cp_depth.at<float>(j,i);
            if(z_real==0) continue;
            float *x_real = new float;
            float *y_real = new float;
            get_point_coord(i,j,z_real,x_real,y_real);
            float x_meter = (*x_real)/1000.0;
            float y_meter = (*y_real)/1000.0;
            float z_meter = z_real/1000.0;
            // printf("(%f, %f, %f)\n", x_meter, y_meter, z_meter);
            o << x_meter << " " << y_meter << " " << z_meter << " ";
            
            int *rgbx = new int;
            int *rgby = new int;
            rgb_point_from_depth(i, j, rgbx, rgby);
            //printf("%d, %d, %d, %d\n", i,j,*rgbx,*rgby);
            if(0<=*rgbx && *rgbx<(rgb->width) && 0<=*rgby && *rgby<(rgb->height)) {
                //unsigned char r = cp_rgb.at<cv::Vec3b>(*rgby,*rgbx)[2];
                //unsigned char g = cp_rgb.at<cv::Vec3b>(*rgby,*rgbx)[1];
                //unsigned char b = cp_rgb.at<cv::Vec3b>(*rgby,*rgbx)[0];
                
                Point3_<uchar>* p = cp_rgb.ptr<Point3_<uchar> >(*rgby,*rgbx);
                uint8_t b = p->x; //B
                uint8_t g = p->y; //G
                uint8_t r = p->z; //R
                //printf("%d, %d, %d\n", r, g, b);
                uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
                uint32_t *rgbintp = &rgb;
                float *rgbfp = (float *)rgbintp;
                float rgbf = *rgbfp;
                o << rgbf << "\n";
            } else {
                o << 0 << "\n";
            }

        }
    }
    printf("Wrote %d points to ",count);
    std::cout << file_name << ".\n";
}

void get_point_coord(int x_on_screen, int y_on_screen, float depth, float *x_real, float *y_real) {
    float result[3];
    result[2] = depth;
    float x = x_on_screen-depth_res_horiz/2;
    int transformed_y = depth_res_vert - y_on_screen;
    float y = transformed_y-depth_res_vert/2;
    float z = depth_focus_to_screen;
    float z_real = depth;
    float x_real_val = x*z_real/z;
    float y_real_val = y*z_real/z;
    *x_real = x_real_val;
    *y_real = y_real_val;
}

int main(int argc, char *argv[])
{
  char* data_file_name = "/home/motion/libfreenect2/examples/protonect/point_cloud_data/data_rgb.pcd";
  if(argc==2) {
    data_file_name = argv[1];
  }
  std::string program_path(argv[0]);
  size_t executable_name_idx = program_path.rfind("Protonect");

  std::string binpath = "/";

  if(executable_name_idx != std::string::npos)
  {
    binpath = program_path.substr(0, executable_name_idx);
  }

  glfwInit();

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

  if(dev == 0)
  {
    std::cout << "no device connected or failure opening the default one!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  protonect_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  
  std::cout.setf(std::ios::fixed, std:: ios::floatfield);
  std::cout.precision(0);
  
  namedWindow("depth",1);
  setMouseCallback("depth",mouse_click_event,NULL);
  
  namedWindow("rgb",1);
  setMouseCallback("rgb",rgb_mouse_click_event,NULL);
  
  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    rgb = frames[libfreenect2::Frame::Color];
    //libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    depth = frames[libfreenect2::Frame::Depth];
    cv::Mat depthMat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
    //cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
    cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);

    
    int key = cv::waitKey(1);
    if((char)key=='d') {
        make_point_cloud_file(data_file_name, "/home/motion/libfreenect2/examples/protonect/point_cloud_data/data_rgb.pcd.bmp");
    }
    protonect_shutdown = protonect_shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape

    listener.release(frames);
    //libfreenect2::this_thread::sleep_for(libfreenect2::chrono::milliseconds(100));
    //sleep(1);
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  return 0;
}

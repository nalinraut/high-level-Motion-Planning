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

void mouse_click_event(int event, int x, int y, int flags, void *userdata) {
  if(event==EVENT_LBUTTONDOWN) {
    Mat cp = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    float total = 0;
    float count = 0;
    //cout << cp.channels() << "\n";
    //cout << abi::__cxa_demangle(typeid(depth->data).name(),0,0,0) << " printed\n";
    //std::cout << cp << "\n";
    int idx = (depth->width*y+x)*depth->bytes_per_pixel;
    /*
    for(int i=x-5;i<x+5;i++) {
      for(int j=y-5;j<y+5;j++) {
        std::cout << (int)cp.at<unsigned char>(i,j) << " ";
        total += cp.at<char>(i,j);
        count ++;
      }
      std::cout << "\n";
    }
    float depth_val = total/count;
    */
    //unsigned int d = *((unsigned int*)(depth->data+idx));
    //unsigned int d = depth->data[idx] | (depth->data[idx+1] << 8) | (depth->data[idx+2] << 16) | (depth->data[idx+3] << 24);
    float z_real = *((float*)(depth->data+idx));
    //float f2 = cp.at<float>(y,x);
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
  }
}

void make_point_cloud_file(char *file_name="/home/motion/libfreenect2/examples/protonect/point_cloud_data/data.pcd") {
    std::cout << "Writing to file: " << file_name << "\n";
    Mat cp = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    int count = 0;
    //ofstream f;
    //f.open("/home/motion/libfreenect2/examples/protonect/point_cloud_data/tmp");
    for(int i=0;i<depth_res_horiz;i++) {
        for(int j=0;j<depth_res_vert;j++) {
            float z_real = cp.at<float>(j,i);
            if(z_real!=0) count++;
        }
    }
    //f.close();
    ofstream o;
    o.open(file_name);
    o << "# .PCD v.7 - Point Cloud Data file format\n" << "VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n";
    o << "COUNT 1 1 1\nWIDTH " << count << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n" << "POINTS " << count << "\nDATA ascii\n";
    for(int i=0;i<depth_res_horiz;i++) {
        for(int j=0;j<depth_res_vert;j++) {
            float z_real = cp.at<float>(j,i);
            if(z_real==0) continue;
            float *x_real = new float;
            float *y_real = new float;
            get_point_coord(i,j,z_real,x_real,y_real);
            float x_meter = (*x_real)/1000.0;
            float y_meter = (*y_real)/1000.0;
            float z_meter = z_real/1000.0;
            // printf("(%f, %f, %f)\n", x_meter, y_meter, z_meter);
            o << x_meter << " " << y_meter << " " << z_meter << "\n";
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
  char* data_file_name = "/home/motion/libfreenect2/examples/protonect/point_cloud_data/data.pcd";
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
  
  while(!protonect_shutdown)
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    depth = frames[libfreenect2::Frame::Depth];
    cv::Mat depthMat = cv::Mat(depth->height, depth->width, CV_32FC1, depth->data);
    //cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
    //cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
    cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
    cv::Mat smallMat;
    cv::Mat ref = cv::Mat::ones(20,20,CV_32F);
    cv::resize(depthMat, smallMat, ref.size());
    
    int key = cv::waitKey(1);
    if((char)key=='d') {
        make_point_cloud_file(data_file_name);
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

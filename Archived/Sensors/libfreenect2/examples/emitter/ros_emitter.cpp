#include <iostream>
#include <signal.h>
#include <stdio.h>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

//for glfwInit
#include <GLFW/glfw3.h>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>

using namespace std;

bool emitter_shutdown = false;

void sigint_handler(int s)
{
  printf("Interrupt called, shutting down ROS emitter\n");
  emitter_shutdown = true;
}

void ros_frame_pack(sensor_msgs::Image& msg,libfreenect2::Frame* frame,libfreenect2::Frame::Type type)
{
  msg.header.seq += 1;
  msg.header.stamp = ros::Time::now();
  msg.width=frame->width;
  msg.height=frame->height;
  msg.is_bigendian = true;
  msg.step = frame->width*frame->bytes_per_pixel;
  size_t numbytes = frame->width*frame->height*frame->bytes_per_pixel;
  msg.data.resize(numbytes);
  copy(frame->data,frame->data+numbytes,msg.data.begin());
  if(type == libfreenect2::Frame::Color) {
    //unsigned char * 3, RGB format
    assert(frame->bytes_per_pixel==3);
    //msg.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    msg.encoding = sensor_msgs::image_encodings::BGR8;
  }
  else {
    //float * 1, F format
    assert(frame->bytes_per_pixel==4);
    msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  }
}

float depth_fov_horiz = 70.6;
float depth_fov_vert = 60;
float depth_res_horiz = 512;
float depth_res_vert = 424;
float depth_focus_to_screen = 364; // THIS CONSTANT WILL CHANGE IF ABOVE FOUR PARAMETERS CHANGE

float rgb_res_horiz = 1920;
float rgb_res_vert = 1080;
float rgb_focus_to_screenx = 900;
float rgb_focus_to_screeny = 900;

//baseline coordinates of rgb relative to depth sensor
float rgb_depth_xoffset = -160;
float rgb_depth_yoffset = 0;
float rgb_depth_zoffset = 0;

void get_point_coord(int x_on_screen, int y_on_screen, float depth, float *x_real, float *y_real) {
    float x = x_on_screen-depth_res_horiz*0.5;
    int transformed_y = depth_res_vert - y_on_screen;
    float y = transformed_y-depth_res_vert*0.5;
    static const float xscale = 1.0/depth_focus_to_screen;
    static const float yscale = 1.0/depth_focus_to_screen;
    float z_real = depth;
    float x_real_val = x*z_real*xscale;
    float y_real_val = y*z_real*yscale;
    *x_real = x_real_val;
    *y_real = y_real_val;
}

// Hard-coded homography matrix. Calculated by calibrate3d.py in homography_calibration using correspondences_3d.txt
double H [3][3]= {{ 995.425712942 , -21.1829434837 , 0.781981838215 },
                  { -5.18953437629 , -992.808413707 , -0.0857717225436 },
                  { -0.0191464612195 , 0.0166070833903 , 0.940724621322 }};
double J[3] = { 45.3658303533*1000 , 2.64994616094*1000 , 0.000627415036891*1000 };
double ox = 978.382486379 , oy = 553.549090254 ;

double H11 = H[0][0];
double H12 = H[0][1];
double H13 = H[0][2];
double H21 = H[1][0];
double H22 = H[1][1];
double H23 = H[1][2];
double H31 = H[2][0];
double H32 = H[2][1];
double H33 = H[2][2];

// Calculate Hx where x is [depthx; depthy; 1]. 
bool rgb_point_from_depth(float xd,float yd,float zd, float &xrgb, float &yrgb) {
    float newx = H11*xd+H12*yd+H13*zd + J[0];
    float newy = H21*xd+H22*yd+H23*zd + J[1];
    float newz = H31*xd+H32*yd+H33*zd + J[2];
    float newx_norm = (newx/newz);
    float newy_norm = (newy/newz);
    xrgb = newx_norm + ox;
    yrgb = newy_norm + oy;
    if(xrgb < 0 || xrgb >= rgb_res_horiz) return false;
    if(yrgb < 0 || yrgb >= rgb_res_vert) return false;
    return true;
}

bool depth_point_to_rgb_coord(float xd,float yd,float zd,float& xrgb,float& yrgb)
{
  float invd  = 1.0/(zd-rgb_depth_zoffset);
  xrgb = (xd-rgb_depth_xoffset)*invd*rgb_focus_to_screenx;
  xrgb += rgb_res_horiz*0.5;
  yrgb = (yd-rgb_depth_yoffset)*invd*rgb_focus_to_screeny;
  yrgb = rgb_res_vert - (yrgb + rgb_res_vert*0.5);
  if(xrgb < 0 || xrgb >= rgb_res_horiz) return false;
  if(yrgb < 0 || yrgb >= rgb_res_vert) return false;
  return true;
}

void ros_point_cloud_make(sensor_msgs::PointCloud2& msg,libfreenect2::Frame* depth) {
  msg.header.seq += 1;
  msg.header.stamp = ros::Time::now();
  msg.width=depth->width;
  msg.height=depth->height;
  msg.fields.resize(3);
  msg.fields[0].name="x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name="y";
  msg.fields[1].offset = sizeof(float);
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name="z";
  msg.fields[2].offset = sizeof(float)*2;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;

  msg.point_step = sizeof(float)*3;
  msg.row_step = sizeof(float)*3*depth->width;
  msg.is_bigendian = true;
  msg.is_dense = false;
  msg.data.resize(msg.row_step*msg.height);
  float* msgdata = reinterpret_cast<float*>(&msg.data[0]);
  for(int i=0;i<depth->width;i++) {
    for(int j=0;j<depth->height;j++,msgdata+=3) {
      int ofs = (j*depth->width+i)*depth->bytes_per_pixel;
      float* dat = reinterpret_cast<float*>(&depth->data[ofs]);
      float z_mm = *dat;
      if(z_mm==0) {
	msgdata[0] = msgdata[1] = msgdata[2] = 0;
      }
      else {
	float x_mm;
	float y_mm;
	get_point_coord(i,j,z_mm,&x_mm,&y_mm);
	float x_meter = x_mm*1e-3;
	float y_meter = y_mm*1e-3;
	float z_meter = z_mm*1e-3;
	msgdata[0] = x_meter;
	msgdata[1] = y_meter;
	msgdata[2] = z_meter;
      }
    }
  }
}

void ros_rgb_point_cloud_make(sensor_msgs::PointCloud2& msg,libfreenect2::Frame* depth,libfreenect2::Frame* rgb) {
  msg.header.seq += 1;
  msg.header.stamp = ros::Time::now();
  msg.width=depth->width;
  msg.height=depth->height;
  msg.fields.resize(6);
  msg.fields[0].name="x";
  msg.fields[0].offset = 0;
  msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[0].count = 1;
  msg.fields[1].name="y";
  msg.fields[1].offset = sizeof(float);
  msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[1].count = 1;
  msg.fields[2].name="z";
  msg.fields[2].offset = sizeof(float)*2;
  msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[2].count = 1;
  /*
  //packed RGB
  msg.fields[3].name="rgb";
  msg.fields[3].offset = sizeof(float)*3;
  msg.fields[3].datatype = sensor_msgs::PointField::UINT8;
  msg.fields[3].count = 3;
  */
  msg.fields[3].name="r";
  msg.fields[3].offset = sizeof(float)*3;
  msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[3].count = 1;
  msg.fields[4].name="g";
  msg.fields[4].offset = sizeof(float)*4;
  msg.fields[4].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[4].count = 1;
  msg.fields[5].name="b";
  msg.fields[5].offset = sizeof(float)*5;
  msg.fields[5].datatype = sensor_msgs::PointField::FLOAT32;
  msg.fields[5].count = 1;

  //packed RGB
  //msg.point_step = sizeof(float)*3+3
  msg.point_step = sizeof(float)*3+3*sizeof(float);
  msg.row_step = msg.point_step*depth->width;
  msg.is_bigendian = true;
  msg.is_dense = false;
  msg.data.resize(msg.row_step*msg.height);
  unsigned char* msgdata = &msg.data[0];
  for(int i=0;i<depth->width;i++) {
    for(int j=0;j<depth->height;j++,msgdata+=msg.point_step) {
      float* xyzdata = reinterpret_cast<float*>(msgdata);
      //packed RGB
      //unsigned char* rgbdata = msgdata+sizeof(float)*3;
      float* rgbdata = reinterpret_cast<float*>(msgdata+sizeof(float)*3);
      int ofs = (j*depth->width+i)*depth->bytes_per_pixel;
      float* dat = reinterpret_cast<float*>(&depth->data[ofs]);
      float z_mm = *dat;
      if(z_mm==0) {
	xyzdata[0] = xyzdata[1] = xyzdata[2] = 0;
	rgbdata[0] = rgbdata[1] = rgbdata[2] = 0;
      }
      else {
	float x_mm;
	float y_mm;
	get_point_coord(i,j,z_mm,&x_mm,&y_mm);
	float x_meter = x_mm*1e-3;
	float y_meter = y_mm*1e-3;
	float z_meter = z_mm*1e-3;
	xyzdata[0] = x_meter;
	xyzdata[1] = -y_meter;
	xyzdata[2] = z_meter;
	float xrgb,yrgb;
	//bool found = depth_point_to_rgb_coord(x_mm,y_mm,z_mm,xrgb,yrgb);
	bool found = rgb_point_from_depth(x_mm,y_mm,z_mm,xrgb,yrgb);
	/*
	if(!found) {
	  //packed RGB
	  //rgbdata[0] = rgbdata[2] = 0xff;
	  rgbdata[0] = rgbdata[2] = 1;
	  rgbdata[1] = 0;
	}
	else*/ {
	  int irgb=(int)xrgb,jrgb=(int)yrgb;
	  if(irgb < 0) irgb = 0;
	  if(irgb >= rgb->width) irgb = rgb->width-1;
	  if(jrgb < 0) jrgb = 0;
	  if(jrgb >= rgb->height) jrgb = rgb->height-1;
	  unsigned char* dat = &rgb->data[(irgb+jrgb*rgb->width)*rgb->bytes_per_pixel];
	  /*
	  //packed RGB
	  rgbdata[0] = dat[2];
	  rgbdata[1] = dat[1];
	  rgbdata[2] = dat[0];
	  */
	  rgbdata[0] = dat[2]/255.0;
	  rgbdata[1] = dat[1]/255.0;
	  rgbdata[2] = dat[0]/255.0;
	}
      }
    }
  }
}

int main(int argc, char *argv[])
{
  glfwInit();

  libfreenect2::Freenect2 freenect2;
  libfreenect2::Freenect2Device *dev = freenect2.openDefaultDevice();

  if(dev == 0)
  {
    std::cout << "no device connected or failure opening the default one!" << std::endl;
    return -1;
  }

  signal(SIGINT,sigint_handler);
  emitter_shutdown = false;

  libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
  libfreenect2::FrameMap frames;

  dev->setColorFrameListener(&listener);
  dev->setIrAndDepthFrameListener(&listener);
  dev->start();

  //initialize ROS and publish messages
  ros::init(argc, argv, "Kinect2_ROS_Emitter", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Publisher rgb_topic = nh.advertise<sensor_msgs::Image>(string("/kinect2/rgb"),1,false);
  ros::Publisher ir_topic = nh.advertise<sensor_msgs::Image>(string("/kinect2/ir"),1,false);
  ros::Publisher depth_topic = nh.advertise<sensor_msgs::Image>(string("/kinect2/depth"),1,false);
  ros::Publisher pc_topic = nh.advertise<sensor_msgs::PointCloud2>(string("/kinect2/pc"),1,false);
  sensor_msgs::Image rgb_img,ir_img,depth_img;
  sensor_msgs::PointCloud2 pc;
  rgb_img.header.seq = ir_img.header.seq = depth_img.header.seq = 0;
  pc.header.seq = 0;
  pc.header.frame_id = "/kinect2";

  std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
  std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
  
  std::cout.setf(std::ios::fixed, std:: ios::floatfield);
  std::cout.precision(0);
  
  int framecount = 0;
  while(!emitter_shutdown && ros::ok())
  {
    listener.waitForNewFrame(frames);
    libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];   

    printf("\nROS FRAME %d\n\n",framecount);
    ros_frame_pack(rgb_img,rgb,libfreenect2::Frame::Color);
    ros_frame_pack(ir_img,ir,libfreenect2::Frame::Ir);
    ros_frame_pack(depth_img,depth,libfreenect2::Frame::Depth);

    //makes a basic uncolored point cloud
    //ros_point_cloud_make(pc,depth);
    //makes an rgb colored point cloud
    ros_rgb_point_cloud_make(pc,depth,rgb);
    rgb_topic.publish(rgb_img);
    ir_topic.publish(ir_img);
    depth_topic.publish(depth_img);
    pc_topic.publish(pc);

    listener.release(frames);

    framecount ++;
  }

  // TODO: restarting ir stream doesn't work!
  // TODO: bad things will happen, if frame listeners are freed before dev->stop() :(
  dev->stop();
  dev->close();

  return 0;
}

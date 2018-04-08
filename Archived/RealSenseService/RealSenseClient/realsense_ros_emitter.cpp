#include <KrisLibrary/myfile.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/Timer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/CompressedImage.h>
#include <jpeglib.h>
#include <jerror.h>
#include <png.h>
#include <fstream>
using namespace std;

string ros_rgb_topic = "/realsense/rgb";
string ros_depth_topic = "/realsense/depth";
string ros_pc_topic = "/realsense/pc";
string ros_compressed_rgb_topic = "/realsense/rgb/compressed";
string ros_compressed_depth_topic = "/realsense/depth/compressed";
const char* rostopic_prefix = "/realsense";
bool firstC = true;
bool first = true;
bool firstD = true;
bool firstCom = true;
bool firstComD = true;
bool firstPC = true;
bool debug = false;
//static const int packet_size = 1024*1024;
static const int packet_size = 1024*64;



bool IsBigEndian() {
  int n = 1;
  // little endian if true
  if(*(char *)&n == 1) return false;
  return true;
}

bool ReadIntPrependedString(File& file,std::string& buf)
{
  int slen;
  if(!file.ReadData(&slen,4)) {
    fprintf(stderr,"Socket::ReadString read length failed\n");
    return false;
  }
  if(slen < 0) {
    fprintf(stderr,"ReadIntPrependedString read length %d is negative\n",slen);
    return false;
  }
  buf.resize(slen);
  if(!file.ReadData(&buf[0],slen)) {
    fprintf(stderr,"ReadIntPrependedString read string failed\n");
    return false;
  }
  return true;
}


void ReadDataFromInputStream(png_structp png_ptr, png_bytep outBytes, png_size_t byteCountToRead){
  png_voidp io_ptr_local = png_get_io_ptr(png_ptr);
  if(io_ptr_local == NULL){
    //should call png_error()
    return;
  }
  memcpy(outBytes, io_ptr_local, byteCountToRead);
  png_ptr->io_ptr = (png_voidp)(((char*)io_ptr_local)+byteCountToRead);

}


class Loop
{
public:
  Loop(ros::NodeHandle& nh) {
    startTime = ros::Time::now();
    rgb_pub = nh.advertise<sensor_msgs::Image>(ros_rgb_topic,1,false);
    depth_pub = nh.advertise<sensor_msgs::Image>(ros_depth_topic,1,false);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(ros_pc_topic,1,false);
    compressed_rgb_pub = nh.advertise<sensor_msgs::CompressedImage>(ros_compressed_rgb_topic, 1, false);
    compressed_depth_pub = nh.advertise<sensor_msgs::CompressedImage>(ros_compressed_depth_topic, 1, false);
    frames_read = 0;
    bytes_read = 0;
  }
  bool ReadAndProcess(File& file);
  bool ReadImage(AnyCollection& msg,File& datastream);
  bool ReadCompressedImage(AnyCollection& msg, File& datastream);
  bool ReadPC(AnyCollection& msg,File& datastream);
  bool ProcessPC(AnyCollection& msg, File& datastream);
  bool decompressJPG(unsigned char* inStream, unsigned long jpg_size);
  bool decompressPNG(unsigned char* inStream);
  int  receiveData(int size, std::vector<unsigned char> *array, File& datastream, int packet_size);


  ros::Time startTime;
  ros::Publisher rgb_pub, depth_pub;
  ros::Publisher compressed_depth_pub, compressed_rgb_pub;
  ros::Publisher pc_pub;
  sensor_msgs::Image rgb_msg,depth_msg;
  sensor_msgs::CompressedImage compressed_depth_msg, compressed_rgb_msg;
  sensor_msgs::PointCloud2 pc_msg;
  vector<char> pc_data;
  vector<int> uvX, uvY;
  int frames_read;
  int bytes_read;
  unsigned char *bmp_buffer = NULL;
  unsigned short *depth_buffer = NULL;
};


//process a header/data message and send it to the appropriate ros topic
bool Loop::ReadAndProcess(File& file)
{
  AnyCollection header;
  string msg;
  if(!ReadIntPrependedString(file,msg)) {
    return false;
  }
  bytes_read += msg.size();
  frames_read += 1;

  stringstream ss(msg);
  ss>>header;
  if(!ss) {
    printf("Error parsing header from string \"%s\"\n",msg.c_str());
    return false;
  }
  string type,name;
  if(!header["type"].as(type)) {
    printf("Header doesn't have \"type\" field\n");
    return false;
  }
  if(type == "Image") {
    if(!header["name"].as(name)) {
      printf("Header doesn't have \"name\" field\n");
      return false;
    }
    if(name == "rgb") {
      if(!ReadImage(header,file)) return false;
      rgb_pub.publish(rgb_msg);
    }
    else if(name == "depth") {
      if(!ReadImage(header,file)) return false;
      depth_pub.publish(depth_msg);
    }
    else {
      printf ("Unknown image %s\n",name.c_str());
      return false;
    }
  }
  else if(type == "CompressedImage"){
    if(!header["name"].as(name)){
      printf("Header doesn't have \"name\" field\n");
      return false;
    }
    if(name =="rgb" || name == "color"){
      if(!ReadCompressedImage(header, file)) return false;
      compressed_rgb_pub.publish(compressed_rgb_msg);  
    }
    else if(name=="depth"){
      if(!ReadCompressedImage(header, file)) return false;
      compressed_depth_pub.publish(compressed_depth_msg);
    }else{
      printf("Unknowin image %s\n", name.c_str());
      return false;
    }
    
  }


  else if(type == "CompressedPC"){

    if(!ProcessPC(header,file)) return false;
    string format;
    if(!header["format"].as(format)){
      printf("Header doens't have format field\n");
      return false;
    }

    if (format == "jpg,png,uvX,uvY"){
      compressed_depth_pub.publish(compressed_depth_msg);
    }else{
      depth_pub.publish(depth_msg);
    }
    compressed_rgb_pub.publish(compressed_rgb_msg);
    pc_pub.publish(pc_msg);

  }



  else if(type == "PointCloud3D") {
    if(!ReadPC(header,file)) return false;
    pc_pub.publish(pc_msg);
  }
  else {
    printf("Unknown type %s\n",type.c_str());
    return false;
  }
  return true;
}

bool Loop::ReadImage(AnyCollection& msg,File& datastream)
{
  Timer timer;
  FILE* imResults;

  sensor_msgs::Image* rosmsg = NULL;
  string name;
  msg["name"].as(name);
  if(name=="rgb")
    rosmsg = &rgb_msg;
  else
    rosmsg = &depth_msg;
  rosmsg->header.seq = (int)msg["id"];
  rosmsg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  rosmsg->width = msg["width"];
  rosmsg->height = msg["height"];
  string format;
  if(!msg["format"].as(format)) {
    printf("Image message doesn't contain \"format\"\n"); 
  }
  int pixelsize = 0;
  if(format == "rgb8") {
    pixelsize = 3;
    rosmsg->encoding = sensor_msgs::image_encodings::BGR8;
    if(debug){
      if(firstC){
        imResults = fopen("colorResults.txt", "w");
        firstC = false;
      }
      else{
        imResults = fopen("colorResults.txt", "a");
      }
    }
  }
  else if(format == "u16")  {
    pixelsize = 2;
    rosmsg->encoding = sensor_msgs::image_encodings::MONO16;
    if (debug){
      if(firstD){
        imResults = fopen("depthResults.txt", "w");
        firstD = false;
      }
      else{
        imResults = fopen("depthResults.txt", "a");
      }
    }
  }
  if(pixelsize == 0) {
    printf("Invalid format %s\n",format.c_str());
    return false;
  }
  int pitch;
  if(!msg["pitch"].as(pitch)) pitch=rosmsg->width*pixelsize;
  rosmsg->is_bigendian = true;
  rosmsg->step = pitch;
  int size = pitch*rosmsg->height;
  rosmsg->data.resize(size);
  string buf;
  int cnt = 0;
  Timer imTimer;
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return false;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
      printf("Current count: %d\n",cnt);
      printf("Size: %d != buffer length + current %d\n",size,cnt+buf.length());
      return false;
    }
    //printf("%d\n",buf.length());
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return false;
    }
    copy(buf.begin(),buf.end(),&rosmsg->data[cnt]);
    cnt += buf.length();
  }
  bytes_read += cnt;
  if (debug){
    fprintf(imResults, "Image size=%d\n", size);
    fprintf(imResults, "Read Image time=%g\n", imTimer.ElapsedTime());
    fprintf(imResults, "Total method time=%g\n",timer.ElapsedTime());
    fclose(imResults);
  }
  return true;
}

bool Loop::ReadCompressedImage(AnyCollection& msg,File& datastream)
{
  Timer timer;
  FILE* imResults;
  sensor_msgs::CompressedImage* rosmsg = NULL;
  string name;
  msg["name"].as(name);
  if(name=="color" || name =="rgb")
    rosmsg = &compressed_rgb_msg;
    //color message 
  else if(name =="depth"){
    rosmsg = &compressed_depth_msg;
    //depth message
  }
  else{
    printf ("Error, message name not recognized");
    return false;
  }
  rosmsg->header.seq = (int)msg["id"];
  rosmsg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  
  string format;
  if(!msg["format"].as(format)) {
    printf("Image message doesn't contain \"format\"\n"); 
  }
  if(format == "jpeg" || format == "compressed") {
    rosmsg->format = "jpeg";
    if(debug){
      if(firstCom){
        imResults = fopen("comResults.txt", "w");
        firstCom = false;
      }else{
        imResults = fopen("comResults.txt", "a");
      }
    }
  }
  else if(format == "png")  {
    rosmsg->format = "png";
    if(debug){
      if(firstComD){
        imResults = fopen("comDepthResults.txt", "w");
        firstComD = false;
      }else{
        imResults = fopen("comDepthResults.txt", "a");
      }
    }
  }
  else{
    printf("Invalid format %s\n", format.c_str());
    return false;
  }
  int size = msg["size"];
  rosmsg->data.resize(size);
  string buf;

  Timer messageTimer;
  int cnt = 0;
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return false;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
      printf("Current count: %d\n",cnt);
      printf("Size: %d != buffer length + current %d\n",size,cnt+buf.length());
      return false;
    }
    //printf("%d\n",buf.length());
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return false;
    }
    copy(buf.begin(),buf.end(),&rosmsg->data[cnt]);
    cnt += buf.length();
  }
  bytes_read += cnt;
  if(debug){
    fprintf(imResults, "Receive message time = %g\n", messageTimer.ElapsedTime());
    fprintf(imResults, "Total message time = %gs\n",timer.ElapsedTime());
    fclose(imResults);
  }
  
  return true;
}

bool Loop::ReadPC(AnyCollection& msg,File& datastream)
{
  Timer timer;
  FILE* pcResults;
  if(debug){
    if(firstPC){
      firstPC = false;
      pcResults = fopen("pcResults.txt", "w");
    }
    else{
      pcResults = fopen("pcResults.txt", "a");
    }
  }
  pc_msg.header.frame_id = "/pedestal";
  pc_msg.header.seq = (int)msg["id"];
  pc_msg.header.stamp = startTime + ros::Duration(double(msg["time"]));
  pc_msg.width=(int)msg["width"];
  pc_msg.height=(int)msg["height"];
  string properties,format;
  if(!msg["properties"].as(properties) || !msg["format"].as(format)) {
    printf("PointCloud3D message doesn't have properties or format items\n");
    return false;
  }
  if(properties != "xyzrgb") {
    printf("TODO: parse non-xyzrgb point clouds\n");
    return false;
  }
  if(format != "fffccc" && format != "sssccc") {
    printf("TODO: parse non fffccc or sssccc format point clouds\n");
    return false;
  }
  pc_msg.fields.resize(6);
  pc_msg.fields[0].name="x";
  pc_msg.fields[0].offset = 0;
  pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[0].count = 1;
  pc_msg.fields[1].name="y";
  pc_msg.fields[1].offset = sizeof(float);
  pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[1].count = 1;
  pc_msg.fields[2].name="z";
  pc_msg.fields[2].offset = sizeof(float)*2;
  pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[2].count = 1;

  //packed RGB
  pc_msg.fields[3].name="rgb";
  pc_msg.fields[3].offset = sizeof(float)*3;
  pc_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[3].count = 1;

  //packed RGB
  pc_msg.point_step = sizeof(float)*3+4;
  pc_msg.row_step = pc_msg.point_step*pc_msg.width;
  pc_msg.is_bigendian = IsBigEndian();
  pc_msg.is_dense = false;
  pc_msg.data.resize(pc_msg.row_step*pc_msg.height);

  int inpixelsize;
  if(format == "fffccc")
    inpixelsize = 3*4+3;
  else
    inpixelsize = 3*2+3;

  int size = pc_msg.width*pc_msg.height*inpixelsize;
  pc_data.resize(size);
  string buf;
  int cnt = 0;
  Timer readPCTimer;
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return false;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
    }
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return false;
    }
    copy(buf.begin(),buf.end(),&pc_data[cnt]);
    cnt += buf.length();
  }
  bytes_read += cnt;
  if(debug){
    fprintf(pcResults, "ReadPC time=%g\n",readPCTimer.ElapsedTime());
    fprintf(pcResults, "PC data length=%d\n", size);
  }
  int k=0;
  if(format == "fffccc") {
    for(int i=0;i<pc_msg.width;i++) {
      for(int j=0;j<pc_msg.height;j++,k++) {
        //copy the k'th pixel
        char* indata = &pc_data[inpixelsize*k];
        unsigned char* outdata = &pc_msg.data[pc_msg.point_step*k];
        memcpy(outdata,indata,sizeof(float)*3);
        unsigned char b=*(unsigned char*)(indata+12);
        unsigned char g=*(unsigned char*)(indata+13);
        unsigned char r=*(unsigned char*)(indata+14);
        unsigned int rgb =  (r << 16) | (g<<8) | b;
        *(unsigned int*)(outdata+12) = rgb;
      }
    }
  }
  else {
    assert(format == "sssccc");
    float scale;
    if(!msg["scale"].as(scale)) scale = 1.0;
    for(int i=0;i<pc_msg.width;i++) {
      for(int j=0;j<pc_msg.height;j++,k++) {
        //copy the k'th pixel
        char* indata = &pc_data[inpixelsize*k];
        unsigned char* outdata = &pc_msg.data[pc_msg.point_step*k];
        short x=*(short*)(indata+0);
        short y=*(short*)(indata+2);
        short z=*(short*)(indata+4);
        unsigned char b=*(unsigned char*)(indata+6);
        unsigned char g=*(unsigned char*)(indata+7);
        unsigned char r=*(unsigned char*)(indata+8);
        *(float*)(outdata+0) = float(x)*scale;
        *(float*)(outdata+4) = float(y)*scale;
        *(float*)(outdata+8) = float(z)*scale;
        unsigned int rgb = (r << 16) | (g<<8) | b;
        *(unsigned int*)(outdata+12) = rgb;
      }
    }
  }
  if(debug){
    fprintf(pcResults, "ReadPC Method time=%g\n", timer.ElapsedTime());
    fclose(pcResults);    
  }
  return true;
}



bool Loop::ProcessPC(AnyCollection& msg,File& datastream)
{
  Timer timer;
  sensor_msgs::PointCloud2* ros_pc_msg = NULL;
  sensor_msgs::CompressedImage* ros_rgb_msg = NULL;
  sensor_msgs::CompressedImage* ros_depth_com_msg = NULL;
  sensor_msgs::Image* ros_depth_msg = NULL;
  string name;

  /*Message Properties:
  type:
  d_width:
  d_height:
  c_width:
  c_height
  id:
  time:
  properties:
  format: jpg/png,png/raw,uvX/uv,uvY (four elements - each of which has two or fewer options)
  FOVx:
  FOVy: 
  depth_scale: mm
  depth_offset: mm
  bits per depth pixel (bpd):
  size1 - color image size
  size2 - depth image size
  size3 - uvX or uvMap size
  size4 - uvY size or 0
  */

  ros_pc_msg = &pc_msg;
  ros_rgb_msg = &compressed_rgb_msg;

  
  //printf("assiging global variables to local ones\n");

  //TODO: change frame_id to something more dynamic
  ros_pc_msg->header.frame_id = "pedestal";
  ros_pc_msg->header.seq = (int)msg["id"];
  ros_pc_msg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  ros_rgb_msg->header.seq = (int)msg["id"];
  ros_rgb_msg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  

  ros_pc_msg->width=(int)msg["d_width"];
  ros_pc_msg->height=(int)msg["d_height"];
  
  //printf("done with simple assignments\n");
  bool raw_depth = false;

  string properties,format; 
  if(!msg["properties"].as(properties) || !msg["format"].as(format)) {
    printf("Compressed PC message doesn't have properties or format items\n");
    return false;
  }
  if(properties != "xyzrgb") {
    printf("TODO: parse non-xyzrgb point clouds\n");
    return false;
  }
  if(format == "jpg,png,uvX,uvY"){
      ros_depth_com_msg = &compressed_depth_msg;
  }else if (format == "jpg,raw,uvX,uvY"){
      ros_depth_msg = &depth_msg;
      raw_depth = true;
  }else{
    return false;
  }
  
  if(raw_depth){
    ros_depth_msg->header.seq = (int)msg["id"];
    ros_depth_msg->header.stamp = startTime + ros::Duration(double(msg["time"]));
    ros_depth_msg->encoding = sensor_msgs::image_encodings::MONO16;
    ros_depth_msg->width = msg["d_width"];
    ros_depth_msg->height = msg["d_height"];
    ros_depth_msg->is_bigendian = true;
    ros_depth_msg->step = ros_depth_msg->width*2;
  }else{
    ros_depth_com_msg->header.seq = (int)msg["id"];
    ros_depth_com_msg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  }

  ros_pc_msg->fields.resize(6);
  ros_pc_msg->fields[0].name="x";
  ros_pc_msg->fields[0].offset = 0;
  ros_pc_msg->fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  ros_pc_msg->fields[0].count = 1;
  ros_pc_msg->fields[1].name="y";
  ros_pc_msg->fields[1].offset = sizeof(float);
  ros_pc_msg->fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  ros_pc_msg->fields[1].count = 1;
  ros_pc_msg->fields[2].name="z";
  ros_pc_msg->fields[2].offset = sizeof(float)*2;
  ros_pc_msg->fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  ros_pc_msg->fields[2].count = 1;

  //packed RGB
  ros_pc_msg->fields[3].name="rgb";
  ros_pc_msg->fields[3].offset = sizeof(float)*3;
  ros_pc_msg->fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  ros_pc_msg->fields[3].count = 1;

  //packed RGB
  ros_pc_msg->point_step = sizeof(float)*3+4;
  ros_pc_msg->row_step = pc_msg.point_step*pc_msg.width;
  ros_pc_msg->is_bigendian = IsBigEndian();
  ros_pc_msg->is_dense = false;
  ros_pc_msg->data.resize(pc_msg.row_step*pc_msg.height);

  int inpixelsize = 1;//jpgmsg["bpd"];
  int size = (ros_pc_msg->width)*(ros_pc_msg->height)*inpixelsize;

  pc_data.resize(size);
  string buf;
  
  //printf("resized pc_data\n");
  int cnt = 0;
  int size1 = msg["size1"];
  int size2 = msg["size2"];
  int size3 = msg["size3"];
  int size4 = msg["size4"];

  FILE* results;

  if(debug){
    if(first){
      first = false;
      results = fopen("results.txt", "w");
    }else{
      results = fopen("results.txt", "a");
    }
    fprintf(results, "compressed color size =%d\ncompressed depth size =%d\ncompressed uvMapX =%d\n compressed uvMapY=%d\nTotal pc passed byte size=%d\n", size1, size2, size3, size4, size1+size2+size3+size4);
  }

  
  ros_rgb_msg->data.resize(size1);
  vector<unsigned char> uvXBuf, uvYBuf;
  uvXBuf.resize(size3);
  uvYBuf.resize(size4);
  uvX.resize(ros_pc_msg->height*ros_pc_msg->width);
  uvY.resize(ros_pc_msg->height*ros_pc_msg->width);
  
//=======================================================================================================================================
  //Receiving data

  Timer timerDataRecieved;

  int val = receiveData(size1, &(ros_rgb_msg->data), datastream, packet_size);
  if(val == -1){
    printf("Error in receiving color data\n");
  }
   cnt = 0;
  if(raw_depth){
    ros_depth_msg->data.resize(size2);
    val = receiveData(size2, &(ros_depth_msg->data), datastream, packet_size);
    if(val == -1){
      printf("Error in receiving depth data\n");
    }
  }else{
    ros_depth_com_msg->data.resize(size2);
    val = receiveData(size2, &(ros_depth_com_msg->data), datastream, packet_size);
    if(val == -1){
      printf("Error in receiving depth data\n");
    }
  }
  val = receiveData(size3, &uvXBuf, datastream, packet_size);
  if(val == -1){
    printf("Error in receiving uvX\n");
  }  
  val = receiveData(size4, &uvYBuf, datastream, packet_size);
  if(val == -1){
    printf("Error in receiving uvY\n");
  }
  

//==========================================================================================================================================
  //Uncompressing data
  bool png_decomp = false;
  bytes_read += cnt;
  if(debug){
    fprintf(results, "Message received time =%g\n", timerDataRecieved.ElapsedTime());
  }

  //reconstruct images
  Timer timer1;
  decompressJPG(&ros_rgb_msg->data[0], (unsigned long)size1); // goes to bmp_buffer
  if (debug){
    fprintf(results, "Color decompress time =%g\n", timer1.ElapsedTime());
  }
  if(!raw_depth){
    Timer timer2;
    png_decomp = decompressPNG(&ros_depth_com_msg->data[0]); // goes to depth_buffer
    if(debug){
      fprintf(results, "Depth decompress time =%g\n", timer2.ElapsedTime());
    }
  }
  //ros_rgb_msg -> data // contains jpg of image
  //ros_depth_com_msg -> data // contains png of image

  // reconstruct uvmap
  Timer timer3;
  int xCount = 0;
  for(int i = 0; i< (int)(size3/sizeof(int)); i+=2 ){
    // first number is value, second number is duration
    int val = 0;
    val = val | uvXBuf[4*i+3]<<24 | uvXBuf[4*i+2] <<16| ( uvXBuf[4*i+1]<<8 ) | (uvXBuf[4*i] & 0x000000ff);
    int duration = 0;
    duration = duration | (uvXBuf[4*i+7]<<24)| (uvXBuf[4*i+6]<<16) | (uvXBuf[4*i+5]<<8) | (uvXBuf[4*i+4] & 0x000000ff);
    int j = 0;
    while (j < duration){
      uvX[xCount] = val;
      xCount++;
      j++;
    }
  }

  int yCount = 0;
  for(int i = 0; i< (int)(size4/sizeof(int)); i+=2){
    // first number is value, second number is duration
    int val = 0;
    val = val | uvYBuf[4*i+3]<<24 | uvYBuf[4*i+2] <<16| ( uvYBuf[4*i+1]<<8 ) | (uvYBuf[4*i] & 0x000000ff);
    int duration = 0;
    duration = duration | (uvYBuf[4*i+7]<<24)| (uvYBuf[4*i+6]<<16) | (uvYBuf[4*i+5]<<8) | (uvYBuf[4*i+4] & 0x000000ff);

    int j =0;
    while (j < duration){
      uvY[yCount] = val;
      yCount++;
      j++;
    }
  }

  if(debug){
    fprintf(results, "UVMap decompress time =%g\n", timer3.ElapsedTime());
  }


  //======================================================================================================
  //Constucting point cloud
  //do = offset distance
  //ds = scale distance
  //d[i,j] = binary value -> convert into distance using instrinsic camera parameters
  //FOVx = FOV horizontal
  //FOVy = FOV vertical (not necessarily the same value)

  //loop fills in points to point cloud, based on points that exist
  int colorWidth = msg["c_width"];
  int colorHeight = msg["c_height"];
  int k = 0;
  double pi = 3.1415926536;
  double scale, offset;
  if(!msg["depth_scale"].as(scale)) {
    scale = 0.001; //assume values in mm
  }
  if(!msg["depth_offset"].as(offset)) {
    offset = 0;
  }
  float FOVx = (double)msg["FOVx"]*pi/180;
  float FOVy = (double)msg["FOVy"]*pi/180;
  //float FOVx = 72*pi/180.0;
  //float FOVy = 40*pi/180.0;
  float xMultiplier = 1.0/(colorWidth*tan(FOVx/2.0));
  float yMultiplier = 1.0/(colorHeight*tan(FOVy/2.0)); 
  int depthWidth = msg["d_width"];

  //float ds = msg["depth_scale"];
  //float doff = msg["depth_offset"];
  //printf("Given colorWidth: %d colorHeight: %d\n", colorWidth, colorHeight);

  int cx = colorWidth/2;
  int cy = colorHeight/2;

  Timer timerPCBuild; 
  if(png_decomp){
    for(int j=0;j<ros_pc_msg->height;j++) {
      for(int i=0;i<ros_pc_msg->width;i++,k++) {
        //copy the k'th pixel
        unsigned char* outdata = &ros_pc_msg->data[ros_pc_msg->point_step*k];

        //tangent is radians
        float z = 0;
        if(raw_depth){
          z = (ros_depth_msg->data[2*(i+j*depthWidth)])*scale + offset;   
        }else{
          z = (depth_buffer[i+j*ros_pc_msg->width])*scale + offset;   
        }
        float x= 2.0*(i-cx)*xMultiplier*(z);
        float y= 2.0*(j-cy)*yMultiplier*(z);
       
        int colorI = uvX[i*ros_pc_msg->height+j];
        int colorJ = uvY[j*ros_pc_msg->width+i];

        unsigned char r = 0;
        unsigned char g = 0;
        unsigned char b = 0;
        if (!(colorI == -1 || colorJ == -1) ){
          r = bmp_buffer[(colorI+colorJ*colorWidth)*3 + 0];
          g = bmp_buffer[(colorI+colorJ*colorWidth)*3 + 1];
          b = bmp_buffer[(colorI+colorJ*colorWidth)*3 + 2];
        }  
        
        *(float*)(outdata+0) = x;
        *(float*)(outdata+4) = y;
        *(float*)(outdata+8) = z;
        unsigned int rgb = (r << 16) | (g<<8) | b;
        //printf("accessing outdata\n");
        *(unsigned int*)(outdata+12) = rgb;
      }
    }    
  }
  if(debug){
    fprintf(results, "Point Cloud building time =%g\n", timerPCBuild.ElapsedTime());
    fprintf(results, "PointCloud reconstruct time =%g\n", timer1.ElapsedTime());  
  }
  
  free(bmp_buffer);
  bmp_buffer = NULL;
  free(depth_buffer);
  depth_buffer = NULL;

  if(debug){
    fprintf(results, "Total method time=%g\n", timer.ElapsedTime());
    fclose(results);
  }
  
  return true;
}

int Loop::receiveData(int size, std::vector<unsigned char> *array_ptr, File& datastream, int packet_size){

  int cnt = 0;
  string buf;
  std::vector<unsigned char> array;
  array.resize(size);
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return -1;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
    }
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return -1;
    }
    copy(buf.begin(),buf.end(), &array[cnt]);
    //memcpy(array, &buf, buf.length());
    cnt += buf.length();
  }
  bytes_read += cnt;
  *array_ptr = array;
  return bytes_read;
}

bool Loop::decompressJPG(unsigned char* inStream, unsigned long jpg_size){
  //from memory
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;
  cinfo.err = jpeg_std_error(&jerr);
  jpeg_create_decompress(&cinfo);

  jpeg_mem_src(&cinfo, inStream, jpg_size);

  int rc = jpeg_read_header(&cinfo, TRUE);
  
  if(rc!=1){
    printf("File does not seem to be a normal JPEG");
    return false;
  }
  
  int width, height, pixel_size, row_stride;
  unsigned long bmp_size;
  jpeg_start_decompress(&cinfo);
  width = cinfo.output_width;
  height = cinfo.output_height;
  pixel_size = cinfo.output_components;

  bmp_size = width*height*pixel_size;
  if (bmp_buffer != NULL){
    printf("Error, bmp_buffer not freed before being malloced again\n");
    return false;
  }
  bmp_buffer = (unsigned char*) malloc(bmp_size);

  row_stride = width*pixel_size;

  //printf("Color width is: %d\n", width);

  while(cinfo.output_scanline < cinfo.output_height){
    unsigned char *buffer_array[1];
    buffer_array[0] = bmp_buffer + (cinfo.output_scanline)*row_stride;
    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  return true;
  //bmp_buffer contains data

}

bool Loop::decompressPNG(unsigned char* inStream){

  //check png signature

  //unsigned char pngSignature[8];
  int number = 8; // number of bytes to read to check if data is a png (1-8)
  int is_png = !png_sig_cmp(inStream, 0, number);
   //printf("checking signature: is_png is %d\n", is_png);
   if(!is_png)
   {
     printf("Bad png signature\n");
     return false;
  }
  png_structp png_ptr = NULL;
  png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
  if(png_ptr == NULL){
    printf("couldn't create read struct\n");
    return false;
  }

  png_infop info_ptr = NULL;
  info_ptr = png_create_info_struct(png_ptr);

  if(info_ptr == NULL){
    //libpng must free file struct memory - info_ptr wasn't allocated correctly
    printf("couldn't create info struct\n");
    png_destroy_read_struct(&png_ptr, NULL, NULL);
    return false;
  }

  png_set_read_fn(png_ptr, inStream, ReadDataFromInputStream);
  //note png_set_compression_buffer_size affects size of zlib buffer - could be used to speed things up - default is 8192 bytes
  // 0 indicates we haven't incremented any bytes during checking the signature
  png_set_sig_bytes(png_ptr, 0);

  //read from png
  png_read_info(png_ptr, info_ptr);
  png_uint_32 width = 0;
  png_uint_32 height = 0;
  int bitDepth = 0;
  int colorType = -1;

  png_uint_32 retval = png_get_IHDR(png_ptr, info_ptr, &width, &height, &bitDepth, &colorType, NULL, NULL, NULL);
  if(retval!=1){
    printf("couldn't get png information");
    return false;
  }

  //printf("Depth width is: %d\n", width);

  unsigned long depth_size = width*height*bitDepth/8;
  //assumes greyscale - so far
  if (depth_buffer != NULL){
    printf("Error, bmp_buffer not freed before being malloced again\n");
    return false;
  }
  depth_buffer = (unsigned short*)malloc(width*height*bitDepth);
  const png_uint_32 bytesPerRow = png_get_rowbytes(png_ptr, info_ptr);
  png_bytepp row_pointers = new png_bytep[height];
  for(int i =0; i<height; i++){
    row_pointers[i] = (unsigned char*)depth_buffer + i*bytesPerRow;
  }

  png_read_image(png_ptr, row_pointers);
  png_destroy_read_struct(&png_ptr, &info_ptr, NULL);
  return true;

}

int main(int argc,char** argv)
{
  const char* addr = "tcp://192.168.1.128:3457";
  //const char* rostopic_prefix = "/realsense";
  if(argc <= 1) {
    printf("USAGE: RealSense_ROS_Emitter tcp://SERVER_IP_ADDR:PORT\n");
    printf("   Running on %s by default\n",addr);
    printf("   Publishing to %s/{rgb,depth,pc,compressed} by default\n",rostopic_prefix);
  }
  if(argc >= 2) {
    addr = argv[1];
    printf("   Reading from addr %s\n",addr);
  }
  if(argc >= 3) {
    rostopic_prefix = argv[2];
    printf("   Publishing to %s/{rgb,depth,pc,compressed}\n",rostopic_prefix);
    ros_rgb_topic  = string(rostopic_prefix)+"/rgb";
    ros_depth_topic  = string(rostopic_prefix)+"/depth";
    ros_pc_topic  = string(rostopic_prefix)+"/pc";
    ros_compressed_depth_topic = string(rostopic_prefix)+"/depth/compressed";
    ros_compressed_rgb_topic = string(rostopic_prefix)+"/rgb/compressed";   
  }
  if(argc >=4){
    if(0 == strcmp(argv[3], "-d")){
      printf("Debugging\n");
      debug = true;
    }
  }
  File f;
  if(!f.Open(addr,FILEREAD|FILEWRITE)) {
    printf("Unable to open client to %s... did you run \"RealSenseClient %s\"?\n",addr,addr);
    return 1;
  }
  //initialize ROS and publish messages
  string nodename = string("RealSense_ROS_Emitter_")+string(rostopic_prefix);
  ros::init(argc, argv, nodename.c_str(), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  Loop loop(nh);
  Timer timer;
  double lastPrintTime = 0.0;
  while(ros::ok()) {
    if(!loop.ReadAndProcess(f)) {
      printf("Abnormal termination\n");
      return 1;
    }
    if(timer.ElapsedTime() > lastPrintTime + 1.0) {
      double t = timer.ElapsedTime();
      cout<<"Read rate: "<<double(loop.bytes_read)/(t-lastPrintTime)/1024/1024<<"mb/s, "<<float(loop.frames_read)/(t-lastPrintTime)<<" images/s"<<endl;
      loop.bytes_read = 0;
      loop.frames_read = 0;
      lastPrintTime = t;
    }
    ThreadYield();
  }
  printf("Terminated due to ros::ok\n");
  return 0;
}

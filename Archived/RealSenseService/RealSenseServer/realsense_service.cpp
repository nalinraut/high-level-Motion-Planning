#include "pxcsensemanager.h"
#include "pxcprojection.h"
#include "util_render.h"
#include <GLdraw/GLUTNavigationProgram.h>
#include <GLdraw/GeometryAppearance.h>
#include <GLdraw/GL.h>
#include <meshing/PointCloud.h>
#include <geometry/AnyGeometry.h>
#include <utils/threadutils.h>
#include <utils/stringutils.h>
#include <utils/AsyncIO.h>
#include <utils/AnyCollection.h>
#include <sspp/Service.h>
#include <Timer.h>
#include <jpeglib.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <png.h>
#include <pngconf.h>
#include <pnglibconf.h>
#include <thread>
#include <stdlib.h>  
//#include <libavutil/opt.h>
//#include <libavcodec/avcodec.h>
//#include <libavutil/channel_layout.h>
//#include <libavutil/common.h>
//#include <libavutil/imgutils.h>
//#include <libavutil/mathematics.h>
//#include <libavutil/samplefmt.h>

#define INBUF_SIZE 4096
#define AUDIO_INBUF_SIZE 20480
#define AUDIO_REFILL_THRESH 4096


struct mem_encode
{
	char *buffer;
	size_t size;
	bool inUse = false;
};


//void video_encode_example(const char *filename, AVCodecID codec_id, PXCImage::ImageInfo info, PXCImage::ImageData data, int fps)
//{
//	AVCodec *codec;
//	AVCodecContext *c = NULL;
//	int i, ret, x, y, got_output;
//	FILE *f;
//	AVFrame *frame;
//	AVPacket pkt;
//	uint8_t endcode[] = { 0, 0, 1, 0xb7 };
//	printf("Encode video file %s\n", filename);
//	/* find the video encoder */
//	codec = avcodec_find_encoder(codec_id);
//	if (!codec) {
//		fprintf(stderr, "Codec not found\n");
//		exit(1);
//	}
//	c = avcodec_alloc_context3(codec);
//	if (!c) {
//		fprintf(stderr, "Could not allocate video codec context\n");
//		exit(1);
//	}
//	/* put sample parameters */
//	c->bit_rate = 400000;
//	/* resolution must be a multiple of two */
//	c->width = info.width;
//	c->height = info.height;
//	/* frames per second */
//	AVRational av_time_base;
//	av_time_base.num = 1;
//	av_time_base.den = fps;
//	//c->time_base = (AVRational){ 1, 25 };
//	c->time_base = av_time_base;
//	/* emit one intra frame every ten frames
//	* check frame pict_type before passing frame
//	* to encoder, if frame->pict_type is AV_PICTURE_TYPE_I
//	* then gop_size is ignored and the output of encoder
//	* will always be I frame irrespective to gop_size
//	*/
//	c->gop_size = 10;
//	c->max_b_frames = 1;
//	//c->pix_fmt = AV_PIX_FMT_YUV420P;
//	c->pix_fmt = AV_PIX_FMT_GRAY16;
//	if (codec_id == AV_CODEC_ID_H264)
//		av_opt_set(c->priv_data, "preset", "slow", 0);
//	/* open it */
//	if (avcodec_open2(c, codec, NULL) < 0) {
//		fprintf(stderr, "Could not open codec\n");
//		exit(1);
//	}
//	f = fopen(filename, "wb");
//	if (!f) {
//		fprintf(stderr, "Could not open %s\n", filename);
//		exit(1);
//	}
//	frame = av_frame_alloc();
//	if (!frame) {
//		fprintf(stderr, "Could not allocate video frame\n");
//		exit(1);
//	}
//	frame->format = c->pix_fmt;
//	frame->width = c->width;
//	frame->height = c->height;
//	/* the image can be allocated by any means and av_image_alloc() is
//	* just the most convenient way if av_malloc() is to be used */
//	ret = av_image_alloc(frame->data, frame->linesize, c->width, c->height,
//		c->pix_fmt, 32);
//	if (ret < 0) {
//		fprintf(stderr, "Could not allocate raw picture buffer\n");
//		exit(1);
//	}
//	/* encode 1 second of video */
//	for (i = 0; i < 25; i++) {
//		av_init_packet(&pkt);
//		pkt.data = NULL;    // packet data will be allocated by the encoder
//		pkt.size = 0;
//		fflush(stdout);
//		/* prepare a dummy image */
//		/* Y */
//		for (y = 0; y < c->height; y++) {
//			for (x = 0; x < c->width; x++) {
//				frame->data[0][y * frame->linesize[0] + x] = x + y + i * 3;
//			}
//		}
//		/* Cb and Cr */
//		for (y = 0; y < c->height / 2; y++) {
//			for (x = 0; x < c->width / 2; x++) {
//				frame->data[1][y * frame->linesize[1] + x] = 128 + y + i * 2;
//				frame->data[2][y * frame->linesize[2] + x] = 64 + x + i * 5;
//			}
//		}
//		frame->pts = i;
//		/* encode the image */
//		ret = avcodec_encode_video2(c, &pkt, frame, &got_output);
//		if (ret < 0) {
//			fprintf(stderr, "Error encoding frame\n");
//			exit(1);
//		}
//		if (got_output) {
//			printf("Write frame %3d (size=%5d)\n", i, pkt.size);
//			fwrite(pkt.data, 1, pkt.size, f);
//			av_packet_unref(&pkt);
//		}
//	}
//	/* get the delayed frames */
//	for (got_output = 1; got_output; i++) {
//		fflush(stdout);
//		ret = avcodec_encode_video2(c, &pkt, NULL, &got_output);
//		if (ret < 0) {
//			fprintf(stderr, "Error encoding frame\n");
//			exit(1);
//		}
//		if (got_output) {
//			printf("Write frame %3d (size=%5d)\n", i, pkt.size);
//			fwrite(pkt.data, 1, pkt.size, f);
//			av_packet_unref(&pkt);
//		}
//	}
//	/* add sequence end code to have a real MPEG file */
//	fwrite(endcode, 1, sizeof(endcode), f);
//	fclose(f);
//	avcodec_close(c);
//	av_free(c);
//	av_freep(&frame->data[0]);
//	av_frame_free(&frame);
//	printf("\n");
//}


void jpgCompressionHelper(char*fileName, PXCImage::ImageInfo info, PXCImage::ImageData data, bool greyScale)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	FILE *file;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);
	if ((file = fopen(fileName, "wb")) == NULL) {
		fprintf(stderr, "can't open %s\n", fileName);
		return;
	}

	jpeg_stdio_dest(&cinfo, file);

	cinfo.image_width = (int)info.width;      // image width and height, in pixels 
	cinfo.image_height = (int)info.height;
	int row_stride = 0;
	if (greyScale){
		cinfo.input_components = 1;
		cinfo.in_color_space = JCS_GRAYSCALE;
		// realsense depth data is 16bit not 8bit
		row_stride = (int)(info.width * cinfo.input_components);
	}
	else{
		cinfo.input_components = 3;     // # of color components per pixel 
		cinfo.in_color_space = JCS_EXT_BGR;
		row_stride = cinfo.image_width * cinfo.input_components + 16;
	}
	jpeg_set_defaults(&cinfo);
	jpeg_start_compress(&cinfo, TRUE);


	JSAMPROW row_pointer[1];
	JSAMPROW base = data.planes[0];
	//printf("writing to file\n");
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = (&(data.planes[0][cinfo.next_scanline*row_stride]));
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}
	jpeg_finish_compress(&cinfo);
	fclose(file);
	jpeg_destroy_compress(&cinfo);
	//printf("jpeg compress done\n");
}

void jpgCompressionHelperNoFile(struct mem_encode* myBuf, PXCImage::ImageInfo info, PXCImage::ImageData data, bool greyScale)
{
	struct jpeg_compress_struct cinfo;
	struct jpeg_error_mgr jerr;
	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_compress(&cinfo);

	unsigned char* instream = NULL;

	unsigned long buf_size = 0;
	myBuf->inUse = true;
	jpeg_mem_dest(&cinfo, &instream, &buf_size);
	
	//jpeg_stdio_dest(&cinfo, file);
	cinfo.image_width = (int)info.width;      // image width and height, in pixels 
	cinfo.image_height = (int)info.height;
	int row_stride = 0;
	if (greyScale){
		cinfo.input_components = 1;
		cinfo.in_color_space = JCS_GRAYSCALE;
		// realsense depth data is 16bit not 8bit
		row_stride = (int)(info.width * cinfo.input_components);
	}
	else{
		cinfo.input_components = 3;     // # of color components per pixel 
		cinfo.in_color_space = JCS_EXT_BGR;
		row_stride = cinfo.image_width * cinfo.input_components + 16;
	}
	jpeg_set_defaults(&cinfo);
	jpeg_start_compress(&cinfo, TRUE);
	JSAMPROW row_pointer[1];
	JSAMPROW base = data.planes[0];
	while (cinfo.next_scanline < cinfo.image_height) {
		row_pointer[0] = (&(data.planes[0][cinfo.next_scanline*row_stride]));
		jpeg_write_scanlines(&cinfo, row_pointer, 1);
	}
	jpeg_finish_compress(&cinfo);
	myBuf->buffer = (char*)instream;
	jpeg_destroy_compress(&cinfo);
	myBuf->size = (int)buf_size;
}


bool pngCompressionHelper(char*fileName, PXCImage::ImageInfo info, PXCImage::ImageData data, bool depth){

	FILE* file;
	if ((file = fopen(fileName, "wb")) == NULL) {
		fprintf(stderr, "can't open %s\n", fileName);
		return false;
	}
	png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	//version string, (png_voidp)user_error_ptr,user_error_fn, user_warning_fn);
	if (!png_ptr) return false;
	//return (ERROR);
	png_infop info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr){
		png_destroy_write_struct(&png_ptr, &info_ptr);
		return false;	//return (ERROR);
	}
	// created png_ptr and info_ptr

	// step 2
	png_init_io(png_ptr, file);

	if (setjmp(png_jmpbuf(png_ptr))){
		printf("Error in initializing i/o");
		return false;
	}
	if (depth){
		png_set_IHDR(png_ptr, info_ptr, (int)info.width, (int)info.height, 16, PNG_COLOR_TYPE_GRAY,
			PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	}
	else{
		//printf("No plans for non depth compression yet\n");
		return false;
	}
	png_write_info(png_ptr, info_ptr);
	png_set_compression_level(png_ptr, 1); // apparently 3-6 are nearly as good at compression and can do so much faster - default is 6

	for (int i = 0; i < (int)info.height; i++){
		//printf("On interation %d\n", i);
		png_write_row(png_ptr, &data.planes[0][(int)info.width*i * 2]);
		// - for realsense data, data.planes[][] is byte addressable - one depth value is 16bits = 2bytes
	}
	png_write_end(png_ptr, info_ptr);
	png_destroy_write_struct(&png_ptr, &info_ptr);
	fclose(file);
	//printf("png compress done\n");
	return true;
}

void WriteDataToPNGStream(png_structp png_ptr, png_bytep data, png_size_t length){
	//png_voidp io_ptr_local = png_get_io_ptr(png_ptr);
	struct mem_encode* p = (struct mem_encode*)png_get_io_ptr(png_ptr);
	size_t nsize = p->size + length;

	if (p->inUse){
		p->buffer = (char*)realloc(p->buffer, nsize);
	}
	else{
		p->buffer = (char*)malloc(nsize);
		p->inUse = true;
	}
	if(!p->inUse){
		printf("error with buffering\n");
		return;
	}
	memcpy(p->buffer + p->size, data, length);
	p->size += length;
	//printf("buffer size = %d\n", p->size);
	//printf("buffer location = %p\n", p->buffer);
	//png_ptr->io_ptr = (io_ptr_local);
}
void FlushDataFromPNGStream(png_structp png_ptr){
	printf("Called flush, but flush is empty\n");
	return;
}

bool pngCompressionHelperNoFile(struct mem_encode* png_ref, PXCImage::ImageInfo info, PXCImage::ImageData data, bool depth){
	png_structp png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
	//version string, (png_voidp)user_error_ptr,user_error_fn, user_warning_fn);
	if (!png_ptr) return false;
	png_infop info_ptr = png_create_info_struct(png_ptr);
	if (!info_ptr){
		png_destroy_write_struct(&png_ptr, &info_ptr);
		return false;	//return (ERROR);
	}
	// created png_ptr and info_ptr

	// step 2
	//png_init_io(png_ptr, depth_buffer);
	//png_set_write_fn(png_structp write_ptr,	voidp write_io_ptr, png_rw_ptr write_data_fn,png_flush_ptr output_flush_fn);
	struct mem_encode p;
	p.buffer = NULL;
	p.size = 0;
	//p.inUse = true;
	
	png_set_write_fn(png_ptr, &p, WriteDataToPNGStream , FlushDataFromPNGStream);
	if (setjmp(png_jmpbuf(png_ptr))){
		printf("Error in initializing i/o");
		return false;
	}
	//printf("about to set IHDR\n");
	if (depth){
		png_set_IHDR(png_ptr, info_ptr, (int)info.width, (int)info.height, 16, PNG_COLOR_TYPE_GRAY,
			PNG_INTERLACE_NONE, PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);
	}
	else{
		//printf("No plans for non depth compression yet\n");
		return false;
	}
	png_write_info(png_ptr, info_ptr);
	png_set_compression_level(png_ptr, 1); // apparently 3-6 are nearly as good at compression and can do so much faster - default is 6 & lower is faster

	for (int i = 0; i < (int)info.height; i++){
		png_write_row(png_ptr, &data.planes[0][(int)info.width*i * 2]);
		// - for realsense data, data.planes[][] is byte addressable - one depth value is 16bits = 2bytes
	}
	png_write_end(png_ptr, info_ptr);

	png_destroy_write_struct(&png_ptr, &info_ptr);
	*png_ref = p;
	//printf("png compress done\n");
	return true;
}

void uvMapCompressionHelper(PXCImage* depth, PXCImage::ImageInfo infod, PXCImage::ImageInfo infoc,
	std::vector<int> *xUVMapBuf_ptr, std::vector<int> *yUVMapBuf_ptr, int* xIndex_ptr, int* yIndex_ptr, std::vector<PXCPointF32> uvmap){
	int oldA = 0; int oldB = 0; int aCount = 0; int bCount = 0;
	int *xDummy = new int[(int)infod.width*(int)infod.height * 2];
	int *yDummy = new int[(int)infod.width*(int)infod.height * 2];
	std::vector<int> xUVMapBuf;
	std::vector<int> yUVMapBuf;
	int xIndex = 0;
	int yIndex = 0;
	for (int y = 0; y < (int)infod.height; y++){
		for (int x = 0; x < (int)infod.width; x++){
			int dind = y*infod.width + x;
			int b = (int)(uvmap[dind].y*infoc.height);
			if (b<0 || b>infoc.height){
				b = -1;
			}
			if (b == oldB){
				bCount++;
				if (bCount == 32000){
					yDummy[yIndex] = oldB;
					yDummy[yIndex + 1] = bCount;
					yIndex += 2;
					bCount = 0;
				}
			}
			else{
				yDummy[yIndex] = oldB;
				yDummy[yIndex + 1] = bCount;
				oldB = b;
				yIndex += 2;
				bCount = 1;
			}
		}
	}
	yDummy[yIndex] = oldB;
	yDummy[yIndex + 1] = bCount;
	yIndex += 2;
	//two loops for better compression
	for (int x = 0; x < (int)infod.width; x++){
		for (int y = 0; y < (int)infod.height; y++){
			int dind = y*infod.width + x;
			int a = (int)(uvmap[dind].x*infoc.width);
			// if out of bounds, we don't care about the point
			// allows for better compression anyways

			if (a<0 || a>infoc.width){
				a = -1;
			}
			if (a == oldA){
				aCount++;
				if (aCount == 32000){
					xDummy[xIndex] = oldA;
					xDummy[xIndex + 1] = aCount;
					xIndex += 2;
					aCount = 0;
				}
			}
			else{
				xDummy[xIndex] = oldA;
				xDummy[xIndex + 1] = aCount;
				oldA = a;
				xIndex += 2;
				aCount = 1;
			}
		}
	}
	xDummy[xIndex] = oldA;
	xDummy[xIndex + 1] = aCount;
	xIndex += 2;
	xUVMapBuf.resize(xIndex);
	std::copy(&xDummy[0], &xDummy[xIndex], &xUVMapBuf[0]);
	delete (xDummy);
	yUVMapBuf.resize(yIndex);
	std::copy(&yDummy[0], &yDummy[yIndex], &yUVMapBuf[0]);
	delete (yDummy);
	*xUVMapBuf_ptr = xUVMapBuf;
	*yUVMapBuf_ptr = yUVMapBuf;
	*xIndex_ptr = xIndex;
	*yIndex_ptr = yIndex;
	//printf("uvMap compress done\n");
}

void BusyWait(double secs)
{
	Timer timer;
	while (timer.ElapsedTime() < secs);
}

void YieldWait(double secs)
{
	Timer timer;
	while (timer.ElapsedTime() < secs) ThreadYield();
}

void PreciseSleep(double secs)
{
	//ASSUMING WINDOWS MINIMUM SLEEP RESOLUTION 40ms
	const static double minSleepTime = 0.040;
	const static double minYieldTime = 0.010;
	//if (secs < minYieldTime) BusyWait(secs);
	//else if (secs < minSleepTime) YieldWait(secs);
	//else ThreadSleep(secs);
	ThreadSleep(secs);
}

bool first = true;

class GeometryViewer : public GLUTNavigationProgram
{
public:
	GeometryViewer()
	{
		draw_geom = &geom1;
		camera.dist = 2.0;
		viewport.n = 0.1;
		viewport.f = 20;
		lastTime = 0;
		refresh = false;
	}
	virtual bool Initialize()
	{
		if (!GLUTNavigationProgram::Initialize()) return false;
		SleepIdleCallback(0);
		return true;
	}
	virtual void RenderWorld()
	{
		Timer timer;
		ScopedLock lock(mutex);
		timer.Reset();
		glPushMatrix();
		Math3D::Matrix4 rotyz;
		rotyz.setZero();
		rotyz(0, 0) = 1;
		rotyz(3, 3) = 1;
		rotyz(1, 2) = 1;
		rotyz(2, 1) = -1;
		glMultMatrixd(rotyz);
		if (draw_geom->type == Geometry::AnyGeometry3D::PointCloud) {
			Meshing::PointCloud3D& pc = draw_geom->AsPointCloud();
			glDisable(GL_LIGHTING);
			glBegin(GL_POINTS);
			for (size_t i = 0; i < pc.points.size(); i++) {
				int col = (int)pc.properties[i][0];
				glColor3ub((col >> 16) & 0xff, (col >> 8) & 0xff, col & 0xff);
				glVertex3dv(pc.points[i]);
			}
			glEnd();
		}
		else
			//this has some overhead for a rapidly updating geometry
			appearance.DrawGL();
		glPopMatrix();
	}
	Geometry::AnyGeometry3D* LockGeometry()
	{
		//double buffered
		if (draw_geom == &geom1) return &geom2;
		else return &geom1;
	}
	void UnlockGeometry(Geometry::AnyGeometry3D* ptr)
	{
		double t = globalTimer.ElapsedTime();
		//printf("Time since last update: %g\n",t-lastTime);
		lastTime = t;

		ScopedLock lock(mutex);
		assert(ptr != draw_geom);
		draw_geom = ptr;
		Math3D::AABB3D bbox = ptr->GetAABB();
		//printf("Size: [%g,%g]x[%g,%g]x[%g,%g]\n",bbox.bmin.x,bbox.bmax.x,bbox.bmin.y,bbox.bmax.y,bbox.bmin.z,bbox.bmax.z);
		appearance.Set(*draw_geom);
		Refresh();
	}
	virtual void Handle_Idle()
	{
		SleepIdleCallback(0);
	}

	Mutex mutex;
	Geometry::AnyGeometry3D* draw_geom;
	Geometry::AnyGeometry3D geom1, geom2;
	GLDraw::GeometryAppearance appearance;
	double lastTime;
	Timer globalTimer;
	bool refresh;
};

class RawPipeline
{
public:
	RawPipeline(bool _visualize = false, GeometryViewer* viewer = NULL, SocketPipeWorker* service = NULL)
		: visualize(_visualize), projection(NULL), m_viewer(viewer), m_service(service), m_color_render(L"Color"), m_depth_render(L"Depth"), m_total_frames(0), m_next_transmit_time(0),
		sendRGB(true), sendDepth(true), sendPC(true), sendCompress(true), rawDepth(true), debug(false)
	{
	}
	~RawPipeline() {
		if (projection) {
			fprintf(stderr, "Warning, RawPipeline closed without Close() being called first\n");
			projection->Release();
		}
	}
	void Close() {
		if (projection) projection->Release();
		projection = NULL;
	}
	bool Init(PXCSenseManager* pp)
	{
		PXCVideoModule::DataDesc desc = {};
		desc.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH;
		desc.streams.color.sizeMax.width = 848;
		desc.streams.color.sizeMax.height = 480;
		if (pp->EnableStreams(&desc) < PXC_STATUS_NO_ERROR) return false;
		return true;
	}
	bool Open(PXCSenseManager* pp) {
		if (projection) projection->Release();
		PXCCaptureManager* cm = pp->QueryCaptureManager();
		if (cm == NULL) {
			fprintf(stderr, "Capture manager could not be queried\n");
			return false;
		}
		PXCCapture::Device* device = cm->QueryDevice();
		if (device == NULL) {
			fprintf(stderr, "Device could not be queried\n");
			return false;
		}
		PXCPointF32 depth_fov;
		//device->QueryPropertyAsPoint(PXCCapture::Device::PROPERTY_DEPTH_FIELD_OF_VIEW, &depth_fov);
		depth_fov = device->QueryDepthFieldOfView();

		projection = device->CreateProjection();
		if (projection == NULL) {
			fprintf(stderr, "Projection could not be created\n");
			return false;
		}
		return true;
	}

	double sendData(int len, char* data, int chunksize, double waitTime, int maxDelay){
		double sleeptime =0;
		std::string chunk;
		for (int i = 0; i < len; i += chunksize) {
			int left = Min(chunksize, len - i);
			chunk.resize(left);
			std::copy(data + i, data + i + left, &chunk[0]);
			m_service->Send(chunk);
			if (m_service->writer.msgQueue.size() > maxDelay) {
				Timer timer;
				PreciseSleep(waitTime);
				sleeptime += timer.ElapsedTime();
			}
		}

		return sleeptime;
	}

	virtual bool OnNewFrame(PXCSenseManager* pp)
	{

		FILE* results;
		if (debug){
			if (first){
				first = false;
				results = fopen("results.txt", "w");
			}
			else{
				results = fopen("results.txt", "a");
			}
		}
		
		const static double waitTime = 0.002;
		if (m_service && m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
			const static int maxDelay = 50;
			double sleeptime = 0;
			while (m_service->writer.msgQueue.size() > maxDelay) {
				Timer timer;
				PreciseSleep(waitTime);
				sleeptime += timer.ElapsedTime();
			}
			if (sleeptime > 0) printf("Waited %gs for flush of write queue\n", sleeptime);
		}
		PXCCapture::Sample* sample = pp->QuerySample();
		PXCCapture::Device* device= pp->QueryCaptureManager()->QueryDevice();
		PXCPointF32 FOV = device->QueryDepthFieldOfView();
		PXCRangeF32 range = device->QueryDepthSensorRange();
		PXCImage* color = sample->color;
		PXCImage* depth = sample->depth;
		std::vector<int> xUVMapBuf;
		std::vector<int> yUVMapBuf;
		int xIndex = 0, yIndex = 0;
		std::stringstream xUVMapStream;
		std::stringstream yUVMapStream;
		struct mem_encode depthBuffer;
		struct mem_encode colorBuffer;
		//buffers
		PXCImage::ImageInfo infoc, infod;
		static std::string jpgBufferRGB;
		
		//TODO: Replace files with pointers
		char* dummyFileName = "dummyFile.jpg";
		char* dummyFileDepthName = "dummyFileDepth.png";

		infoc = color->QueryInfo();
		infod = depth->QueryInfo();
		PXCImage::ImageData datac, datad;
		pxcStatus res = color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB24, &datac);
		if (res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire access to color\n");	return false; }
		res = depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_DEPTH, &datad);
		if (res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire acccess to depth\n"); return false; }
		if (m_viewer || (m_service && m_service->Connected())) {
			//make point cloud
			Timer timer;
			if (sendPC && !sendCompress){
				uvmap.resize(infod.width*infod.height);
				vertices.resize(infod.width*infod.height);
				pxcStatus sts1 = projection->QueryUVMap(depth, &uvmap[0]);
				pxcStatus sts2 = projection->QueryVertices(depth, &vertices[0]);

				if (sts1 >= PXC_STATUS_NO_ERROR && sts2 >= PXC_STATUS_NO_ERROR) {
					Geometry::AnyGeometry3D* geom;
					if (m_viewer)
						geom = m_viewer->LockGeometry();
					else
						geom = &tempGeometry;
					if (geom->type != Geometry::AnyGeometry3D::PointCloud)
						*geom = Meshing::PointCloud3D();
					Meshing::PointCloud3D& pc = geom->AsPointCloud();
					pc.points.resize(infod.width*infod.height);
					pc.propertyNames.resize(1, "rgb");
					pc.properties.resize(pc.points.size());
					unsigned char r, g, b;
					int k = 0;
					for (int y = 0; y < (int)infod.height; y++) {
						for (int x = 0; x < (int)infod.width; x++, k++) {
							pc.points[k].x = vertices[k].x*0.001;
							pc.points[k].y = -vertices[k].y*0.001;
							pc.points[k].z = vertices[k].z*0.001;
							int dind = y*infod.width + x;
							int xx = (int)(uvmap[dind].x*infoc.width);
							int yy = (int)(uvmap[dind].y*infoc.height);
							if (xx >= 0 && xx < (int)infoc.width && yy >= 0 && yy < (int)infoc.height) {
								//is color 3 or 4 bytes?
								int cind = yy*datac.pitches[0] + xx * 3;
								b = datac.planes[0][cind];
								g = datac.planes[0][cind + 1];
								r = datac.planes[0][cind + 2];
							}
							else { r = g = b = 0xff; }
							int rgb = r << 16 | g << 8 | b;
							pc.properties[k].resize(1);
							pc.properties[k][0] = rgb;
						}
					}
					//printf("Time to make point cloud: %g\n",timer.ElapsedTime());
					if (m_viewer) m_viewer->UnlockGeometry(geom);
				}
				else {
					fprintf(stderr, "Unable to query projection values from device\n");
				}
				//fprintf(results, "Time to construct point cloud=%g\n", timer.ElapsedTime());
			}			
		}
		if (sendCompress){
			//Compressing Section ============================================
			//printf("sending compress\n");
			if (sendPC){
				if (rawDepth){
					Timer timer1;
					uvmap.resize(infod.width*infod.height);
					pxcStatus sts1 = projection->QueryUVMap(depth, &uvmap[0]);
					std::thread t1(jpgCompressionHelperNoFile, &colorBuffer, infoc, datac, false);
					std::thread t3(uvMapCompressionHelper, depth, infod, infoc, &xUVMapBuf, &yUVMapBuf, &xIndex, &yIndex, uvmap);
					//TODO: replace with nofile methods
					t3.join();
					t1.join();
					if (debug){
						fprintf(results, "Time to compress all: %g\n", timer1.ElapsedTime());
					}
				}
				else{
					Timer timer1;
					uvmap.resize(infod.width*infod.height);

					pxcStatus sts1 = projection->QueryUVMap(depth, &uvmap[0]);
					//std::thread t1(jpgCompressionHelper, dummyFileName, infoc, datac, false);
					std::thread t1(jpgCompressionHelperNoFile, &colorBuffer, infoc, datac, false);
					//std::thread t2(pngCompressionHelper, dummyFileDepthName, infod, datad, true);
					std::thread t2(pngCompressionHelperNoFile, &depthBuffer, infod, datad, true);
					// in progress
					std::thread t3(uvMapCompressionHelper, depth, infod, infoc, &xUVMapBuf, &yUVMapBuf, &xIndex, &yIndex, uvmap);
					//TODO: replace with nofile methods
					t3.join();
					t2.join();
					t1.join();
					if (debug){
						fprintf(results, "Time to compress all: %g\n", timer1.ElapsedTime());
					}					
				}			
			}
			else{
				if (sendRGB && sendDepth && !rawDepth){
					std::thread t1(jpgCompressionHelperNoFile, &colorBuffer, infoc, datac, false);
					//std::thread t1(jpgCompressionHelper, dummyFileName, infoc, datac, false);
					std::thread t2(pngCompressionHelperNoFile, &depthBuffer, infod, datad, true);
					//std::thread t2(pngCompressionHelper, dummyFileDepthName, infod, datad, true);

					t1.join();
					t2.join();
					
				}
				else{
					if (sendRGB){
						jpgCompressionHelperNoFile(&colorBuffer, infoc, datac, false);
					}
					if (sendDepth && !rawDepth){
						pngCompressionHelperNoFile(&depthBuffer, infod, datad, true);
					}
				}
			}
		}
		if (m_service) {
			//if client was disconnected, stop sending
			if (m_service->Connected() && !m_service->WriteReady())
				m_service->writer.msgQueue.clear();

			if (m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
				double startTime = m_timer.ElapsedTime();
				Timer timer;
				//serialize the images
				AnyCollection colormsg, depthmsg, pcmsg, jpgmsg;
				AnyCollection allmsg;
				//serial queue configuration
				double sleeptime = 0;
				//static const int chunksize = 1024 * 1024;   //1mb messages
				static const int chunksize = 1024 * 64;   //test: 64kb messages
				static const size_t maxDelay = 50;  //will wait until the client is no more than maxDelay chunks behind
				static std::string chunk; chunk.resize(chunksize);
				static const double waitTime = 0.002;

				//Sending Section ===========================================================================================

				if (!sendCompress){ // if I'm sending pc compressed, I don't need to worry about sending rgb and depth - I'm sending those already
					if (sendRGB) {
						//pack the color image
						colormsg["type"] = std::string("Image");
						colormsg["name"] = std::string("rgb");
						colormsg["width"] = infoc.width;
						colormsg["height"] = infoc.height;
						colormsg["pitch"] = datac.pitches[0];
						colormsg["id"] = m_total_frames;
						colormsg["time"] = startTime;
						colormsg["format"] = std::string("rgb8");
						//
						//static std::string base64_str,base64_str2;
						//ToBase64((const char*)datac.planes[0],datac.pitches[0]*infoc.height,base64_str);
						//colormsg["base64_pixels"] = base64_str;
						std::stringstream ss; ss << colormsg; m_service->Send(ss.str());
						int len = datac.pitches[0] * infoc.height;
						sendData(len, (char*)datac.planes[0], chunksize, waitTime, maxDelay);
						printf("Color image pack time %g\n", timer.ElapsedTime());
					}
					if (sendDepth) {
						//pack the depth image
						depthmsg["type"] = std::string("Image");
						depthmsg["name"] = std::string("depth");
						depthmsg["width"] = infod.width;
						depthmsg["height"] = infod.height;
						depthmsg["pitch"] = datad.pitches[0];
						depthmsg["id"] = m_total_frames;
						depthmsg["time"] = startTime;
						depthmsg["format"] = std::string("u16");
						/*
						ToBase64((const char*)datad.planes[0],datad.pitches[0]*infod.height,base64_str2);
						depthmsg["base64_pixels"] = base64_str2;
						*/
						std::stringstream ss; ss << depthmsg; m_service->Send(ss.str());
						int len = (int) (datad.pitches[0] * infod.height);
						sleeptime += sendData(len, (char*)datad.planes[0], chunksize, waitTime, maxDelay);
						printf("Depth image pack time %g\n", timer.ElapsedTime());
					}
					if (sendPC) {
						//pack the point cloud


						Geometry::AnyGeometry3D* geom;
						if (m_viewer) geom = m_viewer->draw_geom;
						else geom = &tempGeometry;
						Meshing::PointCloud3D& pc = geom->AsPointCloud();
						pcmsg["type"] = std::string("PointCloud3D");
						pcmsg["width"] = infod.width;
						pcmsg["height"] = infod.height;
						pcmsg["id"] = m_total_frames;
						pcmsg["time"] = startTime;
						pcmsg["properties"] = std::string("xyzrgb");
						//pcmsg["format"] = std::string("fffccc");
						//save as shorts, in the unit of millimeters
						pcmsg["scale"] = 0.001;
						pcmsg["format"] = std::string("sssccc");
						//int stride = (sizeof(float)*3+sizeof(char)*3);
						int stride = (sizeof(short) * 3 + sizeof(char) * 3);
						int len = pc.points.size()*stride;
						static std::vector<char> buf;
						buf.resize(len);
						char* ptr = &buf[0];
						for (size_t i = 0; i < pc.points.size(); i++, ptr += stride) {

							if (pc.points[i].x == 0 && pc.points[i].y == 0 && pc.points[i].z == 0){
								ptr -= stride;
							}
							else{
								int rgb = (int)pc.properties[i][0];
								//fffccc format
								//*(float*)(ptr+0) = (float)pc.points[i].x;
								//*(float*)(ptr+4) = (float)pc.points[i].y;
								//*(float*)(ptr+8) = (float)pc.points[i].z;
								//*(ptr+12) = (rgb & 0xff);
								//*(ptr+13) = ((rgb >> 8) & 0xff);
								//*(ptr+14) = ((rgb >> 16) & 0xff);
								//sssccc format: saves about 40% of bandwidth
								*(short*)(ptr + 0) = (short)(pc.points[i].x*1000.0);
								*(short*)(ptr + 2) = (short)(pc.points[i].y*1000.0);
								*(short*)(ptr + 4) = (short)(pc.points[i].z*1000.0);
								*(ptr + 6) = (rgb & 0xff);
								*(ptr + 7) = ((rgb >> 8) & 0xff);
								*(ptr + 8) = ((rgb >> 16) & 0xff);
							}
						}

						printf("Point cloud pack time %g\n", timer.ElapsedTime());
						//pcmsg["base64_points"] = ToBase64(&buf[0],len);
						/*
						allmsg["color"] = colormsg;
						allmsg["depth"] = depthmsg;
						allmsg["pc"] = pcmsg;
						printf("Message creation time %g\n",timer.ElapsedTime());
						if(!m_service->Send(allmsg)) {
						printf("Error sending message...\n");
						}
						*/
						std::stringstream ss; ss << pcmsg; m_service->Send(ss.str());
						sleeptime += sendData(len, (char*)&buf[0], chunksize, waitTime, maxDelay);
					}
					printf("Message send time %g\n", timer.ElapsedTime());

				}
				else{
					//sending compressed data
					if (!sendPC){
						if (sendRGB){
							//printf("Entered sendCompress for sending messages");
							jpgmsg["type"] = std::string("CompressedImage");
							jpgmsg["name"] = std::string("color");
							jpgmsg["id"] = m_total_frames;
							jpgmsg["time"] = startTime;
							jpgmsg["format"] = std::string("jpeg");
							jpgmsg["packet_size"] = chunksize;
							std::ifstream fin(dummyFileName, std::ios::in | std::ios::binary);
							std::ostringstream oss;
							oss << fin.rdbuf();
							std::string data(oss.str());

							int len = data.length();
							jpgmsg["size"] = len;
							static const size_t maxDelay = 2;  //will wait until the client is no more than maxDelay images behind
							static std::string chunk; chunk.resize(chunksize);

							std::stringstream ss; ss << jpgmsg; m_service->Send(ss.str());
							sendData(len, (char*)&data[0], chunksize, waitTime, maxDelay);
							printf("Compressed color image pack time%g\n", timer.ElapsedTime());

						}
						if (sendDepth){
							//printf("Entered sendCompress for sending messages");
							jpgmsg["type"] = std::string("CompressedImage");
							jpgmsg["name"] = std::string("depth");
							jpgmsg["id"] = m_total_frames;
							jpgmsg["time"] = startTime;
							jpgmsg["format"] = std::string("png");
							jpgmsg["packet_size"] = chunksize;

							std::ifstream fin(dummyFileDepthName, std::ios::in | std::ios::binary);
							std::ostringstream oss;
							oss << fin.rdbuf();
							std::string data(oss.str());
							int len = data.length();
							jpgmsg["size"] = len;
							static const size_t maxDelay = 50;  //will wait until the client is no more than maxDelay images behind
							static std::string chunk; chunk.resize(chunksize);
							std::stringstream ss; ss << jpgmsg; m_service->Send(ss.str());
							sendData(len, (char*)&data[0], chunksize, waitTime, maxDelay);
							
							printf("Compressed depth image pack time%g\n", timer.ElapsedTime());
						}
					}
					else{
						//send point cloud - includes rgb and depth data
						//printf("sending compress\n");
						pcmsg["type"] = std::string("CompressedPC");
						pcmsg["d_width"] = infod.width;
						pcmsg["d_height"] = infod.height;
						pcmsg["c_width"] = infoc.width;
						pcmsg["c_height"] = infoc.height;
						pcmsg["id"] = m_total_frames;
						pcmsg["time"] = startTime;
						pcmsg["properties"] = std::string("xyzrgb");
						pcmsg["FOVx"] = FOV.y; // i think there was something wrong with the internal ordering...
						pcmsg["FOVy"] = FOV.x;
						pcmsg["depth_scale"] = 0.001; //milimeters
						pcmsg["depth_offset"] = (range.min)*0.001;
						pcmsg["bpd"] = 16; //bits per depth pixel
						std::string format;
						if (rawDepth){
							format = "jpg,raw,uvX,uvY";
						}
						else{
							format = "jpg,png,uvX,uvY";
						}
						pcmsg["format"] = format;
						/*
						Four Cases:
						jpg, raw, uvX, uvY
						jpg, png, uvX, uvY
						jpg, raw, uv
						jpg, uv

						first element is how color is sent
						second element is how depth is sent
						third element is either the compressed form of the uvX map or the compressed form of the total uv map
						fourth element is either the compressed form of the uvY map or nothing
						*/

						bool uvXY = true;
						bool depthCompress = true;
						//std::ifstream fin(dummyFileName, std::ios::in | std::ios::binary);
						//std::ostringstream oss;
						//oss << fin.rdbuf();
						//std::string dataC(oss.str());
						//int len = dataC.length();
						int len = colorBuffer.size;

						std::string dataD;
						std::string colorType, depthType, uvType;

						int a = format.find(","); int b = format.find(",", a+1);
						colorType = format.substr(0, a);
						depthType = format.substr(a+1, b-(a+1));
						uvType = format.substr(b+1);

						if (colorType.compare("jpg") == 0){
							pcmsg["size1"] = len;
						}
						if (depthType.compare("png") == 0){
							/*std::ifstream fin(dummyFileDepthName, std::ios::in | std::ios::binary);
							std::ostringstream oss;
							oss << fin.rdbuf();
							dataD = oss.str();
							int lenDepth = dataD.length();
							pcmsg["size2"] = lenDepth;
							depthCompress = true;
							*/
							pcmsg["size2"] = depthBuffer.size;
							//printf("depth buffer size is %d\n", (int)pcmsg["size2"]);
							depthCompress = true;
						}
						else if (depthType.compare("raw") == 0){
							pcmsg["size2"] = datad.pitches[0] * infod.height;
							depthCompress = false;
						}
						if (uvType.compare("uvX,uvY") == 0){
							pcmsg["size3"] = (int)((xIndex)* sizeof(int));
							pcmsg["size4"] = (int)((yIndex)* sizeof(int));
							uvXY = true;					
						}
						else if (uvType.compare("uv") == 0){
							uvXY = false;
							depthCompress = true;
						}
						else{
							printf("Error, format %s not specified\n", format);
							return false;
						}
	
						//send message
						std::stringstream ss; ss << pcmsg; m_service->Send(ss.str());
						// send jpg
						//sleeptime += sendData(pcmsg["size1"], &dataC[0], chunksize, waitTime, maxDelay);
						sleeptime += sendData(pcmsg["size1"], colorBuffer.buffer, chunksize, waitTime, maxDelay);
						//send depth
						if (!depthCompress){
							sleeptime += sendData(pcmsg["size2"], (char*)datad.planes[0], chunksize, waitTime, maxDelay);
						}
						else{
							//if we are sending depth compressed
							sleeptime += sendData(pcmsg["size2"], (char*)(depthBuffer.buffer), chunksize, waitTime, maxDelay);
							//sleeptime += sendData(pcmsg["size2"], &dataD[0], chunksize, waitTime, maxDelay);
						}
						// send uvMap
						if (uvXY){
							// if in here, we're splitting uvMap into 2
							
							sleeptime += sendData(pcmsg["size3"], (char*)&xUVMapBuf[0], chunksize, waitTime, maxDelay);
							sleeptime += sendData(pcmsg["size4"], (char*)&yUVMapBuf[0], chunksize, waitTime, maxDelay);
						}
						else{
							// send total uvMap
							// not built
						}
					}
				}
				if (sleeptime)
					printf("   %gs spent waiting for write queue / client to catch up\n", sleeptime);

				//don't do this every frame
				double endTime = m_timer.ElapsedTime();
				m_next_transmit_time = endTime + (endTime - startTime);
			}
		}
		//printf("done with pics\n");
		if (depthBuffer.inUse){
			free(depthBuffer.buffer);
			depthBuffer.inUse = false;
		}
		if (colorBuffer.inUse){
			free(colorBuffer.buffer);
			colorBuffer.inUse = false;
		}

		ThreadYield();
		color->ReleaseAccess(&datac);
		depth->ReleaseAccess(&datad);

		if (visualize) {
			bool status = m_color_render.RenderFrame(color);
			if (!status) return false;
			status = m_depth_render.RenderFrame(depth);
			if (!status) return false;
		}
		m_total_frames++;
		if (debug){
			fclose(results);
		}
		return true;
	}

	bool visualize;
	PXCProjection* projection;
	std::vector<PXCPointF32> uvmap;
	std::vector<PXCPoint3DF32> vertices;
	GeometryViewer* m_viewer;
	//SSPP::Service* m_service;
	SocketPipeWorker* m_service;
	UtilRender m_color_render;
	UtilRender m_depth_render;
	int m_total_frames;
	Timer m_timer;
	double m_next_transmit_time;
	Geometry::AnyGeometry3D tempGeometry;
	bool sendRGB, sendDepth, sendPC, sendCompress, rawDepth, debug;
};

//a thread func for running the geometry viewer
void* runGeometryViewer(void* ptr)
{
	GeometryViewer* viewer = reinterpret_cast<GeometryViewer*>(ptr);
	viewer->Run("Point cloud viewer");
	return NULL;
}

//same as pipline.LoopPipeline() but with a thread yield
bool LoopPipelineNice(RawPipeline& pipeline)
{
	PXCSenseManager *pp = PXCSenseManager::CreateInstance();
	if (!pp) {
		fprintf(stderr, "Unable to create the SenseManager\n");
		return false;
	}

	if (!pipeline.Init(pp)) {
		fprintf(stderr, "Error calling Init\n");
		return false;
	}


	pxcStatus sts = pp->Init();
	if (sts < PXC_STATUS_NO_ERROR) {
		fprintf(stderr, "Failed to locate any video stream(s)\n");
		return false;
	}


	if (!pipeline.Open(pp)) {
		fprintf(stderr, "Error calling Init\n");
		return false;
	}

	for (;;) {
		sts = pp->AcquireFrame(false);
		if (sts < PXC_STATUS_NO_ERROR) break;
		if (pp->IsConnected()) { if (!pipeline.OnNewFrame(pp)) break; }
		pp->ReleaseFrame();
		ThreadYield();
	}

	printf("Reader thread terminated with status %d\n", sts);
	pipeline.Close();
	pp->Close();
	pp->Release();
	return true;
}

int main(int argc, const char** argv)
{


	SocketPipeWorker* socket_ptr = NULL;
	if (argc > 1) {
		printf("RealSenseService: hosting server on address %s\n", argv[1]);
		static SocketPipeWorker socket(argv[1], true);
		socket.Start();
		socket_ptr = &socket;
	}
	else {
		printf("RealSenseService: not hosting server\n");
		printf("   Provide the protocol / address on the command line if you want to start\n");
		printf("   a server, e.g., RealSenseServer tcp://localhost:3457\n");
	}

	bool visualize = false;
	bool sendRGB = true, sendDepth = true, sendPC = true, sendCompressed = true, rawDepth = false, debug = false;
	for (int i = 2; i < argc; i++) {
		if (0 == strcmp(argv[i], "-v"))
			visualize = true;
		else if (0 == strcmp(argv[i], "-norgb") || 0 == strcmp(argv[i], "-nocolor"))
			sendRGB = false;
		else if (0 == strcmp(argv[i], "-nodepth"))
			sendDepth = false;
		else if (0 == strcmp(argv[i], "-nopc"))
			sendPC = false;
		else if (0 == strcmp(argv[i], "-nocompressed") || 0 == strcmp(argv[i], "-nocompress")){
			sendCompressed = false;
		}
		else if (0 == strcmp(argv[i], "-d") || 0 == strcmp(argv[i], "-debug")){
			debug = true;
		}
		else if (0 == strcmp(argv[i], "-raw") || 0 == strcmp(argv[i], "-rawdepth")){
			rawDepth = true;
		}
		else {
			fprintf(stderr, "Unknown option %s\n", argv[i]);
			return 1;
		}
	}

	GeometryViewer* viewerPtr = NULL;
	Thread* viewerThreadPtr = NULL;
	if (visualize) {
		printf("Starting point cloud viewer thread...\n");
		static GeometryViewer viewer;
		static Thread viewerThread = ThreadStart(runGeometryViewer, &viewer);
		viewerPtr = &viewer;
		viewerThreadPtr = &viewerThread;
	}
	else
		printf("Not starting point cloud viewer, use -v option if you wish to see the point cloud\n");
	printf("Starting capture pipeline...\n");
	RawPipeline pipeline(visualize, viewerPtr, socket_ptr);
	pipeline.sendRGB = sendRGB;
	pipeline.sendPC = sendPC;
	pipeline.sendDepth = sendDepth;
	pipeline.sendCompress = sendCompressed;
	pipeline.rawDepth = rawDepth;
	pipeline.debug = debug;
	
	if (!LoopPipelineNice(pipeline))
	{
		printf("Error initializing pipeline\n");
		if (viewerThreadPtr) {
			printf("Joining point cloud viewer thread...\n");
			ThreadJoin(*viewerThreadPtr);
		}
		if (socket_ptr) socket_ptr->Stop();
		return 1;
	}

	if (viewerThreadPtr) {
		printf("Joining point cloud viewer thread...\n");
		ThreadJoin(*viewerThreadPtr);
	}
	printf("Exiting normally\n");
	
	if (socket_ptr) socket_ptr->Stop();

	return 0;
}

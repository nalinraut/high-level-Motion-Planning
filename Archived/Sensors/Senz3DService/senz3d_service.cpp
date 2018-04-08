#include "util_pipeline.h"
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
		SleepIdleCallback(0);
	}
	virtual void RenderWorld()
	{
		Timer timer;
		ScopedLock lock(mutex);
		timer.Reset();
		glPushMatrix();
		Math3D::Matrix4 rotyz;
		rotyz.setZero();
		rotyz(0,0) = 1;
		rotyz(3,3) = 1;
		rotyz(1,2) = 1;
		rotyz(2,1) = -1;
		glMultMatrixd(rotyz);
		if(draw_geom->type == Geometry::AnyGeometry3D::PointCloud) {
			Meshing::PointCloud3D& pc = draw_geom->AsPointCloud();
			glDisable(GL_LIGHTING);
			glBegin(GL_POINTS);
			for(size_t i=0;i<pc.points.size();i++) {
				int col = (int)pc.properties[i][0];
				glColor3ub((col >> 16) & 0xff,(col >> 8) &0xff, col&0xff);
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
		if(draw_geom == &geom1) return &geom2;
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
	Geometry::AnyGeometry3D geom1,geom2;
	GLDraw::GeometryAppearance appearance;
	double lastTime;
	Timer globalTimer;
	bool refresh;
};

class RawPipeline: public UtilPipeline
{
public:
	RawPipeline(GeometryViewer* viewer=NULL,SocketPipeWorker* service=NULL)
		:UtilPipeline(), m_viewer(viewer), m_service(service), m_color_render(L"Color"),m_depth_render(L"Depth"), m_total_frames (0), m_next_transmit_time(0)
	{
		EnableImage(PXCImage::IMAGE_TYPE_COLOR, 640, 480);
		EnableImage(PXCImage::IMAGE_TYPE_DEPTH, 320, 240);
	}
	virtual bool OnNewFrame(void) 
	{
		//pre-sleeping
		if(m_service && m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
			const static int maxDelay = 0;
			int sleepnum = 0;
			while(m_service->writer.msgQueue.size() > maxDelay) {
				ThreadSleep(0.02);
				sleepnum ++;
			}
			if(sleepnum) printf("Waited %gs for flush of write queue\n",sleepnum*0.02);
		}

		PXCImage* color = QueryImage(PXCImage::IMAGE_TYPE_COLOR);
		PXCImage* depth = QueryImage(PXCImage::IMAGE_TYPE_DEPTH);

		PXCImage::ImageInfo infoc,infod;
		color->QueryInfo(&infoc);
		depth->QueryInfo(&infod);
		PXCImage::ImageData datac,datad;
		pxcStatus res = color->AcquireAccess(PXCImage::ACCESS_READ,&datac);
		if(res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire access to color\n"); return false; }
		res = depth->AcquireAccess(PXCImage::ACCESS_READ,&datad);
		if(res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire acccess to depth\n"); return false; }

		if(m_viewer) {
			//make point cloud
			//Timer timer;
			Geometry::AnyGeometry3D* geom = m_viewer->LockGeometry();
			if(geom->type != Geometry::AnyGeometry3D::PointCloud)
				*geom = Meshing::PointCloud3D();
			Meshing::PointCloud3D& pc = geom->AsPointCloud();
			pc.points.resize(infod.width*infod.height);
			pc.propertyNames.resize(1,"rgb");
			pc.properties.resize(pc.points.size());
			float *uvmap=(float*)datad.planes[2];
			unsigned short *depthmap = (unsigned short*)datad.planes[0];
			if(uvmap == 0 || depthmap == 0) {
				printf("uvmap or depthmap is NULL!\n");
				return false;
			}
			double fov = 74.0*Math::Pi/180.0;
			int halfw = infod.width/2;
			int halfh = infod.height/2;
			float xscale = float(tan(fov*0.5)/halfw);
			float yscale = float(tan(fov*0.5)/halfw);
			float zscale = 0.001f;  //TODO: calibrate this
			unsigned char r,g,b;
			int k=0;
			for(int y=0;y<(int)infod.height;y++) {
				for(int x=0;x<(int)infod.width;x++,k++) {
					int uvind = (y*infod.width+x)*2; 
					int dind = y*infod.width+x;
					unsigned short depth = depthmap[dind];
					if(depth >= 0 && depth < 65535/5) {
						float fdepth = float(depth) *zscale;
						pc.points[k].x = (x-halfw)*xscale*fdepth;
						pc.points[k].y = (y-halfh)*yscale*fdepth;
						pc.points[k].z = fdepth;
					}
					else {
						pc.points[k].setZero();
					}
					int xx=(int)(uvmap[uvind+0]*infoc.width+0.5);
					int yy=(int)(uvmap[uvind+1]*infoc.height+0.5);
					if(xx>=0 && xx<(int)infoc.width && yy>=0 && yy<(int)infoc.height) {
						int cind = yy*datac.pitches[0]+xx*3;
						b=datac.planes[0][cind];
						g=datac.planes[0][cind+1];
						r=datac.planes[0][cind+2];
					}
					else { r=g=b=0xff; }
					int rgb = r<<16 | g<<8 | b;
					pc.properties[k].resize(1);
					pc.properties[k][0] = rgb;
				}
			}
			//printf("Time to make point cloud: %g\n",timer.ElapsedTime());
			m_viewer->UnlockGeometry(geom);
		}

		//if client was disconnected, stop sending
		if(m_viewer && m_service->Connected() && !m_service->WriteReady())
			m_service->writer.msgQueue.clear();

		if(m_viewer && m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
			double startTime = m_timer.ElapsedTime();
			Timer timer;
			//serialize the images
			AnyCollection colormsg, depthmsg, pcmsg;
			AnyCollection allmsg;
			//pack the color image
			colormsg["type"] = std::string("Image");
			colormsg["name"] = std::string("rgb");
			colormsg["width"] = infoc.width;
			colormsg["height"] = infoc.height;
			colormsg["id"] = m_total_frames;
			colormsg["time"] = startTime;
			colormsg["format"] = std::string("rgb8");
			//
			//static std::string base64_str,base64_str2;
			//ToBase64((const char*)datac.planes[0],datac.pitches[0]*infoc.height,base64_str);
			//colormsg["base64_pixels"] = base64_str;
			int sleepnum = 0;
			static const int chunksize = 1024*1024;
			static const size_t maxDelay = 2;  //will wait until the client is no more than maxDelay images behind
			static std::string chunk; chunk.resize(chunksize);
			{ std::stringstream ss; ss<<colormsg; m_service->SendMessage(ss.str()); 
			int len = datac.pitches[0]*infoc.height;
			for(int i=0;i<len;i+=chunksize) {
				int left = Min(chunksize,len-i);
				chunk.resize(left);
				std::copy(datac.planes[0]+i,datac.planes[0]+i+left,&chunk[0]);
				m_service->SendMessage(chunk);
				if(m_service->writer.msgQueue.size() > maxDelay) {
					ThreadSleep(0.02);
					sleepnum ++;
				}
			}}
			printf("Color image pack time %g\n",timer.ElapsedTime());
			//pack the depth image
			depthmsg["type"] = std::string("Image");
			depthmsg["name"] = std::string("depth");
			depthmsg["width"] = infod.width;
			depthmsg["height"] = infod.height;
			depthmsg["id"] = m_total_frames;
			depthmsg["time"] = startTime;
			depthmsg["format"] = std::string("u16");
			/*
			ToBase64((const char*)datad.planes[0],datad.pitches[0]*infod.height,base64_str2);
			depthmsg["base64_pixels"] = base64_str2;
			*/
			{ std::stringstream ss; ss<<depthmsg; m_service->SendMessage(ss.str()); 
			int len = datad.pitches[0]*infod.height;
			for(int i=0;i<len;i+=chunksize) {
				int left = Min(chunksize,len-i);
				chunk.resize(left);
				std::copy(datad.planes[0]+i,datad.planes[0]+i+left,&chunk[0]);
				m_service->SendMessage(chunk);
				if(m_service->writer.msgQueue.size() > maxDelay) {
					ThreadSleep(0.02);
					sleepnum ++;
				}
			}}
			printf("Depth image pack time %g\n",timer.ElapsedTime());
			//pack the point cloud
			Meshing::PointCloud3D& pc = m_viewer->draw_geom->AsPointCloud();
			pcmsg["type"] = std::string("PointCloud3D");
			pcmsg["width"] = infod.width;
			pcmsg["height"] = infod.height;
			pcmsg["id"] = m_total_frames;
			pcmsg["time"] = startTime;
			pcmsg["properties"] = std::string("xyzrgb");
			pcmsg["format"] = std::string("fffccc");
			int stride = (sizeof(float)*3+sizeof(char)*3);
			int len = pc.points.size()*stride;
			static std::vector<char> buf;
			buf.resize(len);
			char* ptr = &buf[0];
			for(size_t i=0;i<pc.points.size();i++,ptr+=stride) {
				int rgb = (int)pc.properties[i][0];
				*(float*)(ptr+0) = (float)pc.points[i].x;
				*(float*)(ptr+4) = (float)pc.points[i].y;
				*(float*)(ptr+8) = (float)pc.points[i].z;
				*(ptr+12) = (rgb & 0xff);
				*(ptr+13) = ((rgb >> 8) & 0xff);
				*(ptr+14) = ((rgb >> 16) & 0xff);

			}
			//pcmsg["base64_points"] = ToBase64(&buf[0],len);
			/*
			allmsg["color"] = colormsg;
			allmsg["depth"] = depthmsg;
			allmsg["pc"] = pcmsg;
			printf("Message creation time %g\n",timer.ElapsedTime());
			if(!m_service->SendMessage(allmsg)) {
				printf("Error sending message...\n");
			}
			*/
			{ std::stringstream ss; ss<<pcmsg; m_service->SendMessage(ss.str()); 
			for(int i=0;i<len;i+=chunksize) {
				int left = Min(chunksize,len-i);
				chunk.resize(left);
				std::copy(&buf[i],&buf[i]+left,&chunk[0]); 
				m_service->SendMessage(chunk);
				if(m_service->writer.msgQueue.size() > maxDelay) {
					ThreadSleep(0.02);
					sleepnum ++;
				}
			}}
			printf("Message send time %g\n",timer.ElapsedTime());
			if(sleepnum) 
				printf("   %gs spent waiting for write queue / client to catch up\n",0.02*sleepnum);

			//don't do this every frame
			double endTime = m_timer.ElapsedTime();
			m_next_transmit_time = endTime + (endTime - startTime);
		}
		color->ReleaseAccess(&datac);
		depth->ReleaseAccess(&datad);

		bool status = m_color_render.RenderFrame(color);
		if(!status) return false;
		status = m_depth_render.RenderFrame(depth);
		if(!status) return false;
		m_total_frames++;

		return true;
	}

	GeometryViewer* m_viewer;
	//SSPP::Service* m_service;
	SocketPipeWorker* m_service;
	UtilRender m_color_render;
	UtilRender m_depth_render;
	int m_total_frames;
	Timer m_timer;
	double m_next_transmit_time;
};

//a thread func for running the geometry viewer
void* runGeometryViewer(void* ptr)
{
	GeometryViewer* viewer = reinterpret_cast<GeometryViewer*>(ptr);
	viewer->Run("Point cloud viewer");
	return NULL;
}

//same as pipline.LoopPipeline() but with a thread yield
bool LoopPipelineNice(UtilPipeline& pipeline)
{
	bool sts=pipeline.Init();
	if (sts) {
		for (;;) {
			if (!pipeline.AcquireFrame(true)) break;
			if (!pipeline.IsDisconnected()) { if (!pipeline.OnNewFrame()) break; }
			if (!pipeline.ReleaseFrame()) break;
			ThreadYield();
		}
	}
	pipeline.Close();
	return sts;
}

int main(int argc,const char** argv)
{
	SocketPipeWorker* socket_ptr = NULL;
	if(argc > 1) {
		printf("RealSenseService: hosting server on address %s\n",argv[1]);
		static SocketPipeWorker socket(argv[1],true);
		socket.Start();
		socket_ptr = &socket;
	}
	else {
		printf("RealSenseService: not hosting server\n");
		printf("   Provide the protocol / address on the command line if you want to start a server\n");
		printf("   e.g., RealSenseServer tcp://localhost:3457\n");
	}
	GeometryViewer viewer;
	Thread viewerThread = ThreadStart(runGeometryViewer,&viewer);
	RawPipeline pipeline(&viewer,socket_ptr);
	if(!LoopPipelineNice(pipeline)) 
	{
		printf("Error initializing pipeline\n");
		ThreadJoin(viewerThread);
		if(socket_ptr) socket_ptr->Stop();
		return 1;
	}
	ThreadJoin(viewerThread);
	if(socket_ptr) socket_ptr->Stop();
	return 0;
}

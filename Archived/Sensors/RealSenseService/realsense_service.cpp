#include "pxcsensemanager.h"
#include "pxcprojection.h"
#include "util_render.h"
#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/GLdraw/GeometryAppearance.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/geometry/AnyGeometry.h>
#include <KrisLibrary/utils/threadutils.h>
#include <KrisLibrary/utils/stringutils.h>
#include <KrisLibrary/utils/AsyncIO.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <sspp/Service.h>
#include <KrisLibrary/Timer.h>

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
		if(!GLUTNavigationProgram::Initialize()) return false;
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

class RawPipeline
{
public:
	RawPipeline(bool _visualize=false,GeometryViewer* viewer=NULL,SocketPipeWorker* service=NULL)
		: visualize(_visualize),projection(NULL), m_viewer(viewer), m_service(service), m_color_render(L"Color"),m_depth_render(L"Depth"), m_total_frames (0), m_next_transmit_time(0),
		sendRGB(true), sendDepth(true), sendPC(true)
	{
	}
	~RawPipeline() {
		if(projection) {
			fprintf(stderr,"Warning, RawPipeline closed without Close() being called first\n");
			projection->Release();
		}
	}
	void Close() {
		if(projection) projection->Release();
		projection = NULL;
	}
	bool Init(PXCSenseManager* pp)
	{
		PXCVideoModule::DataDesc desc={};
		desc.deviceInfo.streams = PXCCapture::STREAM_TYPE_COLOR | PXCCapture::STREAM_TYPE_DEPTH; 
		if(pp->EnableStreams(&desc) < PXC_STATUS_NO_ERROR) return false;
		return true;
	}
	bool Open(PXCSenseManager* pp) {
		if(projection) projection->Release();
		PXCCaptureManager* cm = pp->QueryCaptureManager();
		if(cm == NULL) {
			fprintf(stderr,"Capture manager could not be queried\n");
			return false;
		}
		PXCCapture::Device* device = cm->QueryDevice();
		if(device == NULL) {
			fprintf(stderr,"Device could not be queried\n");
			return false;
		}
		projection = device->CreateProjection();
		if(projection == NULL) {
			fprintf(stderr,"Projection could not be created\n");
			return false;
		}
		return true;
	}
	virtual bool OnNewFrame(PXCSenseManager* pp) 
	{
		//pre-sleeping
		const static double waitTime = 0.002;
		if(m_service && m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
			const static int maxDelay = 0;
			int sleepnum = 0;
			while(m_service->writer.msgQueue.size() > maxDelay) {
				ThreadSleep(waitTime);
				sleepnum ++;
			}
			if(sleepnum) printf("Waited %gs for flush of write queue\n",sleepnum*waitTime);
		}

		PXCCapture::Sample* sample = pp->QuerySample();
		PXCImage* color = sample->color;
		PXCImage* depth = sample->depth;

		PXCImage::ImageInfo infoc,infod;
		infoc = color->QueryInfo();
		infod = depth->QueryInfo();
		PXCImage::ImageData datac,datad;
		pxcStatus res = color->AcquireAccess(PXCImage::ACCESS_READ,PXCImage::PIXEL_FORMAT_RGB24,&datac);
		if(res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire access to color\n"); return false; }
		res = depth->AcquireAccess(PXCImage::ACCESS_READ,&datad);
		if(res < PXC_STATUS_NO_ERROR) { printf("Unable to acquire acccess to depth\n"); return false; }

		if(m_viewer || (m_service && m_service->Connected())) {
			//make point cloud
			//Timer timer;
			uvmap.resize(infod.width*infod.height);
			vertices.resize(infod.width*infod.height);
			pxcStatus sts1 = projection->QueryUVMap(depth, &uvmap[0]);
			pxcStatus sts2 = projection->QueryVertices(depth,&vertices[0]);
			if(sts1 >= PXC_STATUS_NO_ERROR && sts2 >= PXC_STATUS_NO_ERROR) {
				Geometry::AnyGeometry3D* geom;
				if (m_viewer)
					geom = m_viewer->LockGeometry();
				else
					geom = &tempGeometry;
				if(geom->type != Geometry::AnyGeometry3D::PointCloud)
					*geom = Meshing::PointCloud3D();
				Meshing::PointCloud3D& pc = geom->AsPointCloud();
				pc.points.resize(infod.width*infod.height);
				pc.propertyNames.resize(1,"rgb");
				pc.properties.resize(pc.points.size());
				unsigned char r,g,b;
				int k=0;
				for(int y=0;y<(int)infod.height;y++) {
					for(int x=0;x<(int)infod.width;x++,k++) {
						pc.points[k].x = vertices[k].x*0.001;
						pc.points[k].y = -vertices[k].y*0.001;
						pc.points[k].z = vertices[k].z*0.001;
						int dind = y*infod.width+x;
						int xx=(int)(uvmap[dind].x*infoc.width);
						int yy=(int)(uvmap[dind].y*infoc.height);
						if(xx>=0 && xx<(int)infoc.width && yy>=0 && yy<(int)infoc.height) {
							//is color 3 or 4 bytes?
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
				if(m_viewer) m_viewer->UnlockGeometry(geom);
			}
			else {
				fprintf(stderr,"Unable to query projection values from device\n");
			}
		}
	
		if(m_service) {
			//if client was disconnected, stop sending
			if(m_service->Connected() && !m_service->WriteReady())
				m_service->writer.msgQueue.clear();

			if(m_service->Connected() && m_service->WriteReady() && m_timer.ElapsedTime() >= m_next_transmit_time) {
				double startTime = m_timer.ElapsedTime();
				Timer timer;
				//serialize the images
				AnyCollection colormsg, depthmsg, pcmsg;
				AnyCollection allmsg;
				//serial queue configuration
				int sleepnum = 0;
				static const int chunksize = 1024 * 1024;
				static const size_t maxDelay = 10;  //will wait until the client is no more than maxDelay images behind
				static std::string chunk; chunk.resize(chunksize);
				static const double waitTime = 0.002;
				if (sendRGB) {
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
					{ std::stringstream ss; ss << colormsg; m_service->SendMessage(ss.str());
					int len = datac.pitches[0] * infoc.height;
					for (int i = 0; i < len; i += chunksize) {
						int left = Min(chunksize, len - i);
						chunk.resize(left);
						std::copy(datac.planes[0] + i, datac.planes[0] + i + left, &chunk[0]);
						m_service->SendMessage(chunk);
						if (m_service->writer.msgQueue.size() > maxDelay) {
							ThreadSleep(waitTime);
							sleepnum++;
						}
					}}
					printf("Color image pack time %g\n", timer.ElapsedTime());
				}
				if (sendDepth) {
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
					{ std::stringstream ss; ss << depthmsg; m_service->SendMessage(ss.str());
					int len = datad.pitches[0] * infod.height;
					for (int i = 0; i < len; i += chunksize) {
						int left = Min(chunksize, len - i);
						chunk.resize(left);
						std::copy(datad.planes[0] + i, datad.planes[0] + i + left, &chunk[0]);
						m_service->SendMessage(chunk);
						if (m_service->writer.msgQueue.size() > maxDelay) {
							ThreadSleep(waitTime);
							sleepnum++;
						}
					}}
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
					printf("Point cloud pack time %g\n", timer.ElapsedTime());
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
					{ std::stringstream ss; ss << pcmsg; m_service->SendMessage(ss.str());
					for (int i = 0; i < len; i += chunksize) {
						int left = Min(chunksize, len - i);
						chunk.resize(left);
						std::copy(&buf[i], &buf[i] + left, &chunk[0]);
						m_service->SendMessage(chunk);
						if (m_service->writer.msgQueue.size() > maxDelay) {
							ThreadSleep(waitTime);
							sleepnum++;
						}
					}}
				}
				printf("Message send time %g\n",timer.ElapsedTime());
				if(sleepnum) 
					printf("   %gs spent waiting for write queue / client to catch up\n",waitTime*sleepnum);

				//don't do this every frame
				double endTime = m_timer.ElapsedTime();
				m_next_transmit_time = endTime + (endTime - startTime);
			}
		}
		color->ReleaseAccess(&datac);
		depth->ReleaseAccess(&datad);

		if (visualize) {
			bool status = m_color_render.RenderFrame(color);
			if (!status) return false;
			status = m_depth_render.RenderFrame(depth);
			if (!status) return false;
		}
		m_total_frames++;

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

	bool sendRGB,sendDepth,sendPC;
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
        fprintf(stderr,"Unable to create the SenseManager\n");
        return false;
	}

	if(!pipeline.Init(pp)) {
		fprintf(stderr,"Error calling Init\n");
		return false;
	}


	pxcStatus sts = pp->Init();
    if (sts<PXC_STATUS_NO_ERROR) {
        fprintf(stderr,"Failed to locate any video stream(s)\n");
        return false;
    }


	if(!pipeline.Open(pp)) {
		fprintf(stderr,"Error calling Init\n");
		return false;
	}

	for (;;) {
		sts=pp->AcquireFrame(false);
        if (sts<PXC_STATUS_NO_ERROR) break;
		if (pp->IsConnected()) { if (!pipeline.OnNewFrame(pp)) break; }
		pp->ReleaseFrame();
		ThreadYield();
	}
	pipeline.Close();
	pp->Close();
	pp->Release();
	return true;
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
		printf("   Provide the protocol / address on the command line if you want to start\n");
		printf("   a server, e.g., RealSenseServer tcp://localhost:3457\n");
	}
	bool visualize = false;
	bool sendRGB = true, sendDepth = true, sendPC = true;
	for (int i = 2; i < argc; i++) {
		if (0 == strcmp(argv[i], "-v"))
			visualize = true;
		else if (0 == strcmp(argv[i], "-norgb") || 0 == strcmp(argv[i], "-nocolor"))
			sendRGB = false;
		else if (0 == strcmp(argv[i], "-nodepth"))
			sendDepth = false;
		else if (0 == strcmp(argv[i], "-nopc"))
			sendPC = false;
		else {
			fprintf(stderr, "Unknown option %s\n", argv[i]);
			return 1;
		}
	}
	GeometryViewer* viewerPtr=NULL;
	Thread* viewerThreadPtr=NULL;
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
	RawPipeline pipeline(visualize,viewerPtr,socket_ptr);
	pipeline.sendRGB = sendRGB;
	pipeline.sendPC = sendPC;
	pipeline.sendDepth = sendDepth;
	if(!LoopPipelineNice(pipeline)) 
	{
		printf("Error initializing pipeline\n");
		if (viewerThreadPtr) {
			printf("Joining point cloud viewer thread...\n");
			ThreadJoin(*viewerThreadPtr);
		}
		if(socket_ptr) socket_ptr->Stop();
		return 1;
	}
	if (viewerThreadPtr) {
		printf("Joining point cloud viewer thread...\n");
		ThreadJoin(*viewerThreadPtr);
	}
	if(socket_ptr) socket_ptr->Stop();
	return 0;
}

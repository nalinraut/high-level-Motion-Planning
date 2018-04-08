#include "TaskManager.h"
#include "PyTaskGenerator.h"
#include "Common/system_config.h"
#include <IO/ROS.h>
#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/meshing/PointCloud.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <sspp/Topic.h>

//serial communication address from Klamp't <- ebolabot state server
const static string stateServer_default = "tcp://localhost:4568";

//KH: temporarily disable point cloud visualization
#define DISABLE_POINT_CLOUDS 1

//refresh rate
#define FRAME_RATE 30

//klampt model path
const static string klampt_model_default = "klampt_models/ebolabot_col.rob";

const char* klampt_left_limb_names [] = {
  "left_upper_shoulder",
  "left_lower_shoulder",
  "left_upper_elbow",
  "left_lower_elbow",
  "left_upper_forearm",
  "left_lower_forearm",
  "left_wrist"
};


const char* klampt_right_limb_names [] = {
  "right_upper_shoulder",
  "right_lower_shoulder",
  "right_upper_elbow",
  "right_lower_elbow",
  "right_upper_forearm",
  "right_lower_forearm",
  "right_wrist"
};


struct PointCloudSensor
{
  string name;
  string ros_topic;
  int robot_link;
  int object_index;
  RigidTransform local_transform;
  SmartPointer<Meshing::PointCloud3D> pointCloud;
};

vector<PointCloudSensor> sensors;

PointCloudSensor* AddPointCloudSensor(const char* name,const char* ros_topic,const char* link_name,RobotWorld& world)
{
  Robot& robot = *world.robots[0];
  int link = robot.LinkIndex(link_name);
  if(link < 0) {
    fprintf(stderr,"Warning, %s link %s doesn't exist in robot model\n",name,link_name);
    return NULL;
  }
  RigidObject* obj = new RigidObject;
  /*
  string topicPath = string("ros:PointCloud2//")+string(ros_topic);
  if(!obj->LoadGeometry(topicPath.c_str())) {
    fprintf(stderr,"Warning, was not able to connect %s to ROS topic\n",name);
    delete obj;
    return NULL;
  }
  */
  sensors.resize(sensors.size()+1);
  PointCloudSensor* s = &sensors.back();
  obj->geometry.CreateEmpty();
  *obj->geometry = Geometry::AnyCollisionGeometry3D(Meshing::PointCloud3D());
  s->pointCloud = new Meshing::PointCloud3D();
  if(!ROSSubscribePointCloud(*s->pointCloud,ros_topic)) {
    sensors.resize(sensors.size()-1);
    delete obj;
    return NULL;
  }
  s->name = name;
  s->ros_topic = ros_topic;
  s->robot_link = link;
  s->object_index = world.AddRigidObject(name,obj);
  s->local_transform.setIdentity();
  return s;
}

bool LoadSensorCalibration(RobotWorld& world)
{
  PointCloudSensor* kinect2 = AddPointCloudSensor("Kinect2","/kinect2/sd/points","torso",world);
  if(kinect2) {
    QuaternionRotation kinect2_quaternion(0.566222,0.423455,0.424871,0.5653);
    Vector3 kinect2_position(0.228665,0.0591513,0.0977748);
    kinect2_quaternion.getMatrix(kinect2->local_transform.R);
    kinect2->local_transform.t = kinect2_position;
  }
  PointCloudSensor* lrealsense = AddPointCloudSensor("Left RealSense","/left_realsense/pc","left_gripper",world);
  if(lrealsense) {
    //flip x and y
    lrealsense->local_transform.R.setRotateZ(-Pi*0.5);
    Matrix3 Rtilt; Rtilt.setRotateY(DtoR(-20.0));
    lrealsense->local_transform.R = Rtilt*lrealsense->local_transform.R;
  }
  PointCloudSensor* rrealsense = AddPointCloudSensor("Right RealSense","/right_realsense/pc","right_gripper",world);
  if(rrealsense) {
    //flip x and y
    rrealsense->local_transform.R.setRotateZ(-Pi*0.5);
    Matrix3 Rtilt; Rtilt.setRotateY(DtoR(-20.0));
    rrealsense->local_transform.R = Rtilt*rrealsense->local_transform.R;
  }
  //TODO: calibrate RealSense transforms
  printf("Loaded the following available sensors: \n");
  for(size_t i=0;i<sensors.size();i++)
    printf("  %s (ROS topic %s)\n",sensors[i].name.c_str(),sensors[i].ros_topic.c_str());
  //getchar();
  return true;
}

void swap(Meshing::PointCloud3D& a,Meshing::PointCloud3D& b)
{
  swap(a.points,b.points);
  swap(a.properties,b.properties);
  swap(a.propertyNames,b.propertyNames);
}

void UpdateSensorVisualization(RobotWorld& world,Mutex& worldMutex)
{
  //update ROS topics, if used 
  if(ROSSubscribeUpdate()) {
    //TODO: filtering
    worldMutex.lock();
    //need to refresh geometry / appearances.  Since this is done in another
    //thread, we have to copy geometries / appearances manually
    for(size_t i=0;i<sensors.size();i++) {
      int oindex = sensors[i].object_index;
      if(ROSHadUpdate(sensors[i].ros_topic.c_str())) {
        swap(world.rigidObjects[oindex]->geometry->AsPointCloud(),*sensors[i].pointCloud);
        world.rigidObjects[oindex]->geometry.Appearance()->Set(*world.rigidObjects[oindex]->geometry);
        world.rigidObjects[oindex]->geometry.Appearance()->vertexSize = 2.0;
      }
    }
    worldMutex.unlock();
    /*
    for(size_t i=0;i<world.rigidObjects.size();i++) {
      world.rigidObjects[i]->geometry.DynamicGeometryUpdate();
      world.rigidObjectViews[i]->geometry.Appearance()->vertexSize = 2.0;
    }
    for(size_t i=0;i<world.terrains.size();i++) {
      world.terrains[i]->geometry.DynamicGeometryUpdate();
      world.terrains[i]->geometry.Appearance()->vertexSize = 2.0;
    }
    */
  }
  worldMutex.lock();
  //update the transforms of all the sensor frames
  Robot& robot = *world.robots[0];
  for(size_t i=0;i<sensors.size();i++) {
    world.rigidObjects[sensors[i].object_index]->T = robot.links[sensors[i].robot_link].T_World*sensors[i].local_transform;
  }
  worldMutex.unlock();
}

void* manager_update_thread_func(void* data);

class ManagerUpdater
{
public:
  Thread thread;
  bool kill;

  TaskManager& manager;
  Mutex& mutex;
  SmartPointer<SSPP::MultiTopicListener> systemStateListener;
  AnyCollection *qCmdTopic, *qSnsTopic, *qEndLeftTopic, *qEndRightTopic;
  Config qsns,qcmd,qend;
  int nqsns,nqcmd,nqendleft,nqendright;

  ManagerUpdater(const char* system_state_addr,TaskManager& _manager,Mutex& _mutex)
  : kill(false),manager(_manager),mutex(_mutex) {
    systemStateListener = new SSPP::MultiTopicListener(system_state_addr);
    qcmd = manager.commonWorld.robots[0]->q;
    qend = qcmd;
    qsns = qcmd;
  }
  ~ManagerUpdater() {
    if(!kill) Stop();
  }
  void Start() {
    qSnsTopic = &systemStateListener->Listen(".robot.sensed.q");
    qCmdTopic = &systemStateListener->Listen(".robot.command.q");
    qEndLeftTopic = &systemStateListener->Listen(".controller.left.traj_q_end");
    qEndRightTopic = &systemStateListener->Listen(".controller.right.traj_q_end");
    nqsns = nqcmd = nqendleft = nqendright = 0;
    systemStateListener->OnStart();
    thread = ThreadStart(manager_update_thread_func,this);
  }
  void Stop() {
    kill = true;
    thread.join();
    systemStateListener->OnStop();
    systemStateListener = NULL;
    qSnsTopic = NULL;
    qCmdTopic = NULL;
    qEndLeftTopic = NULL;
    qEndRightTopic = NULL;
  }
  bool Process() {
    //update GUI with commanded configuration from system state service topic
    RobotWorld& world = manager.commonWorld;
    int nm = systemStateListener->Process(); 
    if(nm > 0) {
      vector<double> vqcmd;

	  nqsns += systemStateListener->UnreadCount(".robot.sensed.q");
      nqcmd += systemStateListener->UnreadCount(".robot.command.q");
      nqendleft += systemStateListener->UnreadCount(".controller.left.traj_q_end");
      nqendright += systemStateListener->UnreadCount(".controller.right.traj_q_end");
      //printf("Read %d %d %d messages\n",nqcmd,nqendleft,nqendright);
      if(systemStateListener->UnreadCount(".robot.sensed.q") > 0) {
	if(systemStateListener->Get(".robot.sensed.q").asvector(vqcmd)) {
	  ScopedLock lock(mutex);
	  qsns = Config (vqcmd);
	}
	else {
	  printf("System state reader... error reading sensed config?\n");
	}
	  }
      if(systemStateListener->UnreadCount(".robot.command.q") > 0) {
	if(systemStateListener->Get(".robot.command.q").asvector(vqcmd)) {
	  ScopedLock lock(mutex);
	  qcmd = Config (vqcmd);
	}
	else {
	  printf("System state reader... error reading commanded config?\n");
	}
      }
      //update base DOFs. ignore limb indices
      for(int i=0;i<10;i++)
	qend[i] = qcmd[i];
      for(int i=50;i<qend.n;i++)
        qend[i] = qcmd[i];
      if(systemStateListener->UnreadCount(".controller.left.traj_q_end") > 0) {
	if(systemStateListener->Get(".controller.left.traj_q_end").asvector(vqcmd)) {
	  ScopedLock lock(mutex);
	  for(int i=0;i<7;i++)
	    qend[world.robots[0]->LinkIndex(klampt_left_limb_names[i])] = vqcmd[i];
	}
	else {
	  // printf("System state reader... error reading left end config?\n");
    ScopedLock lock(mutex);
    for(int i=0;i<7;i++) {
      int k=world.robots[0]->LinkIndex(klampt_left_limb_names[i]);
      qend[k] = qcmd[k];
    }
	}
      }
      if(systemStateListener->UnreadCount(".controller.right.traj_q_end") > 0) {
	if(systemStateListener->Get(".controller.right.traj_q_end").asvector(vqcmd)) {
	  ScopedLock lock(mutex);
	  for(int i=0;i<7;i++)
	    qend[world.robots[0]->LinkIndex(klampt_right_limb_names[i])] = vqcmd[i];
	}
	else {
	  //printf("System state reader... error reading right end config?\n");
    ScopedLock lock(mutex);
    for(int i=0;i<7;i++) {
      int k=world.robots[0]->LinkIndex(klampt_right_limb_names[i]);
      qend[k] = qcmd[k];
    }
	}
      }
    }
    if(world.robots[0]->q.n != qcmd.n) {
    	fprintf(stderr,"Warning, robot configuration size does not match commanded configuration size on system state server. Possible mismatch in robot model?\n");
    	fprintf(stderr,"  %d vs %d\n",world.robots[0]->q.n,qcmd.n);
    	return false;
    }
    {
      ScopedLock lock(mutex);
      world.robots[0]->UpdateConfig(qcmd);
      manager.Update();
    }
    return true;
  }
};

void* manager_update_thread_func(void* data)
{
  ManagerUpdater* u = reinterpret_cast<ManagerUpdater*>(data);

  //runs at 50 Hz
  double dt = 0.02;
  Timer timer;
  while(!u->kill) {
    timer.Reset();
    if(!u->Process()) break;
    double sleeptime = Max(dt-timer.ElapsedTime(),0.0);
    ThreadSleep(sleeptime);
  }
  return data;
}

class ManagerVisualizer : public GLUTNavigationProgram
{
public:
  TaskManager& manager;
  Mutex mutex;
  ManagerUpdater updater;
  Timer timer;
  double last_refresh_time;
  vector<string> modes;
  int currentMode;
  bool pluginCapturedClick;
  ManagerVisualizer(const char* system_state_addr,TaskManager& _manager)
  : manager(_manager),updater(system_state_addr,_manager,mutex),last_refresh_time(0) {
    manager.commonWorld.lights.resize(1);
    manager.commonWorld.lights[0].setColor(GLDraw::GLColor(1,1,1));
    manager.commonWorld.lights[0].setDirectionalLight(Vector3(0.4,-0.2,1));
    manager.AvailableModes(modes);
    cout<<"Available modes:"<<endl;
    for(size_t i=0;i<modes.size();i++)
      cout<<"  \""<<modes[i]<<"\""<<endl;
    currentMode = -1;
    if(manager.current) {
      for(size_t i=0;i<modes.size();i++)
        if(manager.current->Name() == modes[i]) {
          currentMode = (int)i;
          break;
        }
    }
    pluginCapturedClick = false;
    updater.Start();
  }
  ~ManagerVisualizer() { updater.Stop(); ROSShutdown(); }
  GLGUIPlugin* GetPlugin() {
    GLGUIPlugin* plugin = manager.GLPlugin();
    if(!plugin) return NULL;
    //camera.toCamera(viewport);
    plugin->viewport = viewport;
    return plugin;
  }
  virtual bool Initialize() {
    if(!GLUTNavigationProgram::Initialize()) return false;
    camera.dist = 6;
    camera.tgt.z = 0.75;
    camera.rot.y = -Math::Pi*0.5;
    viewport.n = 0.1;
    viewport.f = 100;
    viewport.setLensAngle(DtoR(30.0));
    
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glClearColor(manager.commonWorld.background.rgba[0],manager.commonWorld.background.rgba[1],manager.commonWorld.background.rgba[2],manager.commonWorld.background.rgba[3]);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    return true;
  }
  virtual void SetWorldLights() {
    manager.commonWorld.SetGLLights();
  }
  virtual void RenderWorld() {
    RobotWorld& world = manager.commonWorld;
    ScopedLock lock(mutex);
    GLDraw::GLColor transparent(0.5,0.5,0.5,0.5);
    world.robotViews[0].SetColors(transparent);
    glEnable(GL_LIGHTING);
    world.robots[0]->UpdateConfig(updater.qsns);
    world.DrawGL();
    if(!updater.qend.empty()) {
      world.robots[0]->UpdateConfig(updater.qend);
      world.robotViews[0].PushAppearance();
      GLDraw::GLColor green(0,0,1,1);
      world.robotViews[0].SetColors(green);
      world.robotViews[0].Draw();
      world.robotViews[0].PopAppearance();
    }

    GLGUIPlugin* plugin = manager.GLPlugin();
    if(plugin) plugin->RenderWorld();
  }
  virtual void RenderScreen() {
    ScopedLock lock(mutex);
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    void* font = GLUT_BITMAP_HELVETICA_12;
    int x=20,y=20;
    int h=14;
    if(manager.current) {
      glRasterPos2i(x,y);
      glutBitmapString(font,manager.current->Name().c_str());
      y += h;
      glRasterPos2i(x,y);
      glutBitmapString(font,manager.current->Status().c_str());
      y += h;
    }
    GLGUIPlugin* plugin = manager.GLPlugin();
    if(plugin) plugin->RenderScreen();
  }
  virtual void Handle_Idle() {
    GLUTNavigationProgram::Handle_Idle();
    RobotWorld& world = manager.commonWorld;

    //update the sensors
    UpdateSensorVisualization(world,mutex);

    //update the UI
    if(manager.current) {
      ScopedLock(mutex);
      GLGUIPlugin* plugin = GetPlugin();
      if(plugin) {
	plugin->OnIdle();
	RespondToPlugin(plugin);
      }
    }
    //determine whether to refresh or sleep some more
    double ttotal = timer.ElapsedTime();
    if(ttotal > last_refresh_time + 1.0/FRAME_RATE) {
      Refresh();
      last_refresh_time = ttotal;
    }
    else {
      double delay = last_refresh_time + 1.0/FRAME_RATE - ttotal;
      SleepIdleCallback(delay*1000);
    }
  }
  virtual void Handle_Keypress(unsigned char key,int x,int y)
  {
    if(key == 'm') {
      currentMode++;
      if(currentMode >= (int)modes.size()) {
	currentMode = -1;
  ScopedLock(mutex);
  manager.SetMode("");
  StartPlugin(GetPlugin());
      }
      else {
	cout<<"Activating "<<modes[currentMode]<<endl;
  ScopedLock lock(mutex);
  manager.commonWorld.robots[0]->UpdateConfig(updater.qcmd);
	manager.SetMode(modes[currentMode]);
  StartPlugin(GetPlugin());
      }
      Refresh();
    }
    else if(key == 'r') {
      if(currentMode >= 0) {
        printf("Reinitializing mode %s...\n",modes[currentMode].c_str());
        manager.SetMode(modes[currentMode],true);
      }
      else
        printf("No active mode?\n");
    }
    else {
      GLGUIPlugin* plugin = GetPlugin();
      if(plugin) {
        ScopedLock lock(mutex);
	string str; str+=key;
	plugin->OnKeyDown(str);
	RespondToPlugin(plugin);
      }
    }
  }
  virtual void Handle_Reshape(int w,int h) {
    GLUTNavigationProgram::Handle_Reshape(w,h);
    GLGUIPlugin* plugin = GetPlugin();
    if(plugin) {
      ScopedLock lock(mutex);
      plugin->viewport = viewport;
      plugin->OnViewportChange();
      RespondToPlugin(plugin);
    }
  }
  virtual void Handle_Click(int button,int state,int x,int y) {
    GLGUIPlugin* plugin = GetPlugin();
    if(plugin) {
      ScopedLock(mutex);
      if(plugin->OnMouseClick(button,state,x,y)) {
	pluginCapturedClick = true;
	RespondToPlugin(plugin);
	return;
      }
    }
    pluginCapturedClick = false;
    GLUTNavigationProgram::Handle_Click(button,state,x,y);
  }
  virtual void Handle_Drag(int x,int y)
  {
    if(pluginCapturedClick) {
      ScopedLock(mutex);
      GLGUIPlugin* plugin = GetPlugin();
      if(plugin->OnMouseMove(x,y)) {
	RespondToPlugin(plugin);
	return;
      }
    }
    else {
      GLUTNavigationProgram::Handle_Drag(x,y);
    }
  }
  //assumes mutex is locked
  void StartPlugin(GLGUIPlugin* plugin) {
    if(!plugin) return;
    plugin->viewport = viewport;
    plugin->OnViewportChange();
  }
  //assumes mutex is locked
  void RespondToPlugin(GLGUIPlugin* plugin) {
    if(plugin->wantsRefresh) Refresh();
    if(plugin->sleepIdleTime == 0) SleepIdleCallback(0);
    else if(IsInf(plugin->sleepIdleTime)) SleepIdleCallback();
    else SleepIdleCallback(plugin->sleepIdleTime*1000);
    plugin->wantsRefresh = false;
    plugin->sleepIdleTime = 0;
  }
};


bool InitPythonBindings(int argc,char** argv,const char* klampt_model)
{
  //register the Python bindings
  Py_InitializeEx(1);
  //allow python threads to run in the background
  PyEval_InitThreads();

  PySys_SetArgv(argc,argv);
  string scwd;
  char cwd[1024];
  if(getcwd(cwd,sizeof(cwd)) != NULL) 
    scwd = cwd;
  else {
    perror("getcwd() error");
    return false;
  }
  PyObject *sys = PyImport_ImportModule("sys");
  PyObject *path = PyObject_GetAttrString(sys, "path");
  PyList_Append(path, PyString_FromString(scwd.c_str()));
  PyList_Append(path, PyString_FromString((scwd+"/UI/TaskGenerators").c_str()));

  //register the controllers
  RegisterPyTaskGenerators("UI/TaskGenerators");
  return true;
}

int main(int argc,char** argv)
{
  //get some configuration variables
  std::string system_state_addr;
  string klampt_model;
  EbolabotSystemConfig::GetDefault("klampt_model",klampt_model,klampt_model_default);
  if(!EbolabotSystemConfig::Get("state_server_computer_ip",system_state_addr)) {
    printf("Failed to get system state computer IP address\n");
    return 1;
  }

  //register python TaskGenerators
  if(!InitPythonBindings(argc,argv,klampt_model.c_str())) {
    printf("Failed to load python TaskGenerators\n");
    return 1;
  }

  //create the task manager
  TaskManager manager(system_state_addr.c_str());

  //load robot model into the common world
  if(manager.commonWorld.LoadElement(klampt_model.c_str()) < 0) {
    printf("Failed to load Klamp't robot model\n");
    return 1;
  }

#if DISABLE_POINT_CLOUDS
  //KH: temporarily disabling sensors
#else
  //read from kinect and other sensor streams, if available
  if(!LoadSensorCalibration(manager.commonWorld)) {
    printf("Failed to load sensor setup\n");
    return 1;
  }
#endif

  //these are needed to pre-init mouse hovering and TaskGenerator
  //collision detectors.
  manager.commonWorld.InitCollisions();
  manager.commonWorld.UpdateGeometry();

  if(argc > 1) {
    bool res = manager.SetMode(argv[1]);
    if(!res) {
      printf("Warning, requested mode %s not available.  Available modes:\n",argv[1]);
      vector<string> modes;
      manager.AvailableModes(modes);
      for(size_t i=0;i<modes.size();i++)
        printf("  %s\n",modes[i].c_str());
      printf("Press enter to continue, or 'q' to quit.\n");
      int c = getchar();
      if(c == 'q') {
        Py_Finalize();
        return 0;
      }
    }
  }


  //run the visualizer
  ManagerVisualizer visualizer(system_state_addr.c_str(),manager);  

  visualizer.Run("GLUT Task Manager Test");
  ClearPyTaskGenerators();
  Py_Finalize();
  return 0;
}

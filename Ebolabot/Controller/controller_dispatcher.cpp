/** This program provides a dispatcher for operator assistance controllers.
 * This program
 * 1) connects to a Motion server.
 * 2) reads in EbolabotTask messages.
 * 3) dispatches them to the appropriate task controller
 * 4) manages the task controller and provides feedback to the gui
 *
 * RUNNING:
 * 0. Run Ebolabot/Common/system_state_service.py (check the printout to make sure
 *    it's set up on localhost:4568).
 * 1. Start up the robot, connect to it via ROS (cd ~/ros_ws/; . baxter.sh)
 * 2. Run the Motion server (either MotionServer_kinematic or
 *    MotionServer_physical)
 * 3. Run this program.
 * 4. Run your UI (example: python Ebolabot/UI/haptic_widget.py) to start serving
 *    tasks to this program.
 *
 * Subscribed state topics
 * - .controller.task: the requested task
 *
 * Published state topics
 * - .controller.task_status: "invalid" (invalid parameters),
 *   "running" (working on it),  "done" (successfully completed),
 *   "failed" (task stopped unsuccessfully),  "fatal" (fatal error,
 *   cannot accept additional tasks), 
 * - .controller.messages: a list of messages to be fed back to the UI
 *
 * To add tasks in C++:
 * - Subclass TaskController (see TaskController.h)
 * - Instantiate a global member of your subclass and call TaskController::RegisterMe(this) in its constructor
 * - Add your file to the ControllerDispatcher executable.
 * 
 * To add tasks in Python:
 * - Subclass TaskController (see task_controller.py) in a script
 *   XController.py.  By convention, X is the name of the task that your
 *   controller will handle.
 * - Add a make() function to your script
 * - Place your script in Ebolabot/Controller/Tasks
 */
#include "motion.h"
#undef APIENTRY
#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/GLdraw/GLUTString.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/utils/stringutils.h>
#include "Common/system_config.h"
#include "TaskController.h"
#include "PyTaskController.h"
#include <sspp/Topic.h>
#include <sspp/Send.h>
#include <fstream>
#include <unistd.h>

using namespace std;

//serial communication address <- ebolabot motion server
const static string motionServer_default = "tcp://localhost:8001";

//serial communication address from Klamp't <- ebolabot state server
const static string stateServer_default = "tcp://localhost:4568";

//klampt model path
const static string klampt_model_default = "klampt_models/ebolabot_col.rob";

//timeout for task to complete, in seconds
double gTaskTimeout = 10;

/** Usage
 * ControllerDispatcher c();
 * c.RunForever();
 * In an external thread you may call c.Terminate();
 */
class ControllerDispatcher : public SSPP::TopicClient
{
public:
  TaskController* current;
  RobotWorld commonWorld;
  string oldStatus;
  double taskStartTime;

  ControllerDispatcher(const char* stateServer)
    :SSPP::TopicClient(stateServer),
     taskStartTime(-1)
  { 
    //this is a SSPP Service setting... if true, in case the task starts publishing like crazy, only reports the newest
    //I don't think we want to permit skipping tasks
    //onlyProcessNewest = true;

    //clear the current task before subscribing
    AnyCollection empty;
    Set(".controller.task",empty);
    //now subscribe
    Subscribe(".controller.task");
    Subscribe(".robot.t");

    SSPP::TopicClient::tolerateReadErrors = true;
    current = NULL;
  }

  ~ControllerDispatcher() {
    stopMotion();
  }

  virtual const char* Name() const { return "ControllerDispatcher"; }

  virtual const char* Description() const { return "Dispatches Tasks to appropriate controllers "; }

  virtual int Process()
  {
    int n=SSPP::TopicClient::Process();
    string mystatus;
    if(current) {
      string s = current->Status();
      if(s=="ok" || s=="stopping") 
	mystatus = "running";
      else if(s=="invalid parameters")
	mystatus = "invalid";
      else if(s=="done")
	mystatus = "done";
      else if(s=="fatal")
	mystatus = "fatal";
      else if(s=="unavailable")
	mystatus = "failed";
      else if(s=="error" || s=="incomplete")
	mystatus = "failed";
      //cout<<"Status of task controller "<<s<<", my status "<<mystatus<<" (old status: "<<oldStatus<<")"<<endl;
    }
    if(mystatus != oldStatus) {
      AnyCollection msg;
      msg = mystatus;
      Change(".controller.task_status",msg);
      oldStatus = mystatus;
    }
    return n;
  }

  void StopCurrent()
  {
    if(current != NULL) {
      printf("Stopping current task %s...\n",current->TaskName().c_str());
      string status = current->Status();
      if(status=="ok") 
	current->Stop();
	//may need to wait
	while(current->Status()=="stopping") {
	  printf("Waiting for task %s to stop...\n",current->TaskName().c_str());
	  ThreadSleep(0.05);
	}
      current = NULL;
    }
  }

  virtual bool OnTopicMessage(const string& path,AnyCollection& message) { 
    if(path == ".robot.t") {
      //check task timeout
      Real t;
      if(!message.as(t)) {
	printf(".robot.t topic is not a float?\n");
	return false;
      }
      if(current && (current->Status() == "ok")) {
	if(t > taskStartTime + gTaskTimeout) {
	  printf("TASK TIMEOUT REACHED, FORCING IT TO STOP\n");
	  StopCurrent();
	  assert(current == NULL);
	}
      }
    }
    else if(path == ".controller.task") {
      if(message.size()==0) {
	//no task
	if(current) {
	  printf("Got a null task, stopping current task %s\n",current->TaskName().c_str());
	  StopCurrent();
	}
	return true;
      }

      //it's a new task! parse it and set up the correct cspace
      string type;
      if(!message["type"].as(type)) {
	printf("Message does not contain type attribute\n");
	cout<<message<<endl;
	//return false;
	return true;
      }
      double startTime;
      if(!message["task_request_time"].as(startTime)) {
	printf("Message does not contain task_request_time attribute\n");
	cout<<message<<endl;
	//return false;
	return true;
      }
      taskStartTime = startTime;
      
      if(type == "") {
	if(current) {
	  printf("Got a task with empty type, stopping current task %s\n",current->TaskName().c_str());
	  StopCurrent();
	}
	return true;
      }
      if(TaskController::Registry().count(type) == 0) {
	printf("Got an unknown task \"%s\", stopping robot\n",type.c_str());
	stopMotion();
	//return false;
	return true;
      }
      TaskController* c=TaskController::Registry()[type];
      if(c != current) {
	cout<<"Got a new task of type "<<type<<", switching from current"<<endl;
	printf("  Start time %g\n",taskStartTime);
	StopCurrent();
	current = c;
	current->SetWorld(&commonWorld);
      }
      if(!current->Start(message)) {
	printf("Task %s controller returned false on Start()...\n",type.c_str());
      }
      cout<<"Starting TASK "<<message<<endl;
    }
    return true; 
  }
};

#define FRAME_RATE 30
class DispatcherVisualizer : public GLUTNavigationProgram
{
public:
  ControllerDispatcher& dispatcher;
  Timer timer;
  double last_refresh_time;
  DispatcherVisualizer(ControllerDispatcher& _dispatcher): dispatcher(_dispatcher),last_refresh_time(timer.ElapsedTime()) {
    dispatcher.commonWorld.lights.resize(1);
    dispatcher.commonWorld.lights[0].setColor(GLDraw::GLColor(1,1,1));
    dispatcher.commonWorld.lights[0].setDirectionalLight(Vector3(0.2,-0.4,1));

  }

  virtual bool Initialize() {
    if(!GLUTNavigationProgram::Initialize()) return false;
    camera.dist = 6;
    camera.tgt.z = 0.7;
    viewport.n = 0.1;
    viewport.f = 100;
    viewport.setLensAngle(DtoR(30.0));
    
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glClearColor(dispatcher.commonWorld.background.rgba[0],dispatcher.commonWorld.background.rgba[1],dispatcher.commonWorld.background.rgba[2],dispatcher.commonWorld.background.rgba[3]);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    return true;
  }
  virtual void SetWorldLights() {
    dispatcher.commonWorld.SetGLLights();
  }
  virtual void RenderWorld() {
    glEnable(GL_LIGHTING);
    Config q(dispatcher.commonWorld.robots[0]->links.size());
    getKlamptSensedPosition(&q[0]);
    dispatcher.commonWorld.robots[0]->UpdateConfig(q);
    dispatcher.commonWorld.DrawGL();
    if(dispatcher.current) {
      dispatcher.current->DrawGL();
    }
  }
  virtual void RenderScreen() {
    glDisable(GL_LIGHTING);
    glColor3f(0,0,0);
    void* font = GLUT_BITMAP_HELVETICA_12;
    int x=20,y=20;
    int h=14;
    if(dispatcher.current) {
      glRasterPos2i(x,y);
      glutBitmapString(font,dispatcher.current->TaskName().c_str());
      y += h;
      glRasterPos2i(x,y);
      glutBitmapString(font,dispatcher.current->Status().c_str());
      y += h;
    }
  }
  virtual void Handle_Idle() {
    int numMessages;
    do {
      numMessages = dispatcher.Process();
      if(numMessages < 0) {
	printf("Disconnected\n");
	glutDestroyWindow(main_window);
	exit(0);
      }
    } while(numMessages > 0);
    double t = timer.ElapsedTime();
    if(t > last_refresh_time + 1.0/FRAME_RATE) {
      Refresh();
    }
    else {
      double delay = last_refresh_time + 1.0/FRAME_RATE - t;
      SleepIdleCallback(delay*1000);
    }
  }
};

bool InitPythonBindings(int argc,char** argv,const char* klampt_model)
{
  //register the Python bindings
  Py_Initialize();
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
  PyList_Append(path, PyString_FromString((scwd+"/Motion").c_str()));
  PyList_Append(path, PyString_FromString((scwd+"/Controller/Tasks").c_str()));
  //PySys_SetPath("/home/motion/iml-internal/Ebolabot/Motion:/home/motion/iml-internal/Ebolabot/Controller/Tasks");  
  /*
  PyRun_SimpleString( "import sys\n\
import os\n\
home = os.path.expanduser(\"~\")\n\
sys.path.append(os.path.join(home,\"iml-internal/Ebolabot\"))\n\
sys.path.append(os.path.join(home,\"iml-internal/Ebolabot/Motion\"))\n\
sys.path.append(os.path.join(home,\"iml-internal/Ebolabot/Controller/Tasks\"))\n" );
  */

  PyObject* pModule = PyImport_ImportModule("motion");
  if(!pModule) {
    printf("Error opening Python module motion.py\n");
    return 0;
  }
  PyObject* setupFunc = PyObject_GetAttrString(pModule,"setup");
  if(!setupFunc) {
    printf("Python module motion.py does not have setup() function\n");
    Py_DECREF(pModule);
    return 0;
  }
  if (!PyCallable_Check(setupFunc)) {
    printf("Python module motion.py has a make object that is not a function\n");
    Py_DECREF(pModule);
    Py_DECREF(setupFunc);
    return 0;
  }
  PyObject* arglist = Py_BuildValue("(sss)","client","./",klampt_model);
  PyObject* object = PyEval_CallObject(setupFunc, arglist);
  Py_DECREF(arglist);
  Py_DECREF(setupFunc);
  if(object == NULL) {
    PyErr_Print();
    Py_DECREF(pModule);
    return false;
  }
  Py_DECREF(object);
  Py_DECREF(pModule);

  //register the controllers
  RegisterPyTaskControllers("Controller/Tasks");
  return true;
}


int main(int argc, char** argv)
{
  bool visualization = false;
  int argstart = 1;
  if(argc > 1) {
    if(0==strcmp(argv[1],"--help") || 0==strcmp(argv[1],"-h")) {
      printf("USAGE: ControllerDispatcher [world_obstacles]\n");
      return 0;
    }
    if(0==strcmp(argv[1],"--visualization") || 0==strcmp(argv[1],"-v")) {
      visualization = true;
      argstart++;
    }
  }

  string motionServer,stateServer;
  string klampt_model;
  EbolabotSystemConfig::GetDefault("motion_computer_ip",motionServer,motionServer_default);
  EbolabotSystemConfig::GetDefault("klampt_model",klampt_model,klampt_model_default);
  EbolabotSystemConfig::GetDefault("state_server_computer_ip",stateServer,stateServer_default);
	cout<<"Using motion computer ip "<<motionServer<<endl;

  setServerAddr(motionServer.c_str());
  //setServerAddr("192.168.0.101");

  publishState(stateServer.c_str());
 
  if(!setKlamptModel(klampt_model.c_str())) {
    printf("Couldn't load Ebolabot Klampt model from %s, check paths\n",klampt_model.c_str()); 
    return 1;
  }
 
  for(int i=argstart;i<argc;i++) {
    if(!addPlannerObstacle(argv[i])) return 1;
  }

  //send start signal and wait for startup
  printf("Starting up Motion module, server\n");
  sendStartup();
  while(!isStarted()) {
    ThreadSleep(0.5);
    printf("Waiting for Motion module startup...\n");
  }
  printf("Motion module started up.\n");
  printf("Setting end effector tool center points to (-0.04,0,0.15).\n");
  double lofs[3]={-0.03,-0.05,0.10};
  double rofs[3]={0.0,0.08,0.10};

  setEndEffectorOffset(LEFT,lofs);
  setEndEffectorOffset(RIGHT,rofs);

  if(!InitPythonBindings(argc,argv,klampt_model.c_str())) {
    sendShutdown();
    return 1;
  }
  printf("Initializing the controller's world...\n");
  //run the main control loop
  ControllerDispatcher controller(stateServer.c_str());
  controller.commonWorld.LoadElement(klampt_model.c_str());
  for(int i=argstart;i<argc;i++) 
    controller.commonWorld.LoadElement(argv[1]);    
  controller.sleepTime = 0.001;
  printf("Done.\n");

  printf("\n");
  printf("Available tasks:\n");
  for(std::map<std::string,TaskController*>::iterator i = TaskController::Registry().begin(); i!=TaskController::Registry().end(); i++) {
    printf("  \"%s\"\n",i->first.c_str());
  }
  printf("\n");

  if(visualization) {
    printf("Running controller dispatcher server loop with visualizer...\n");
    controller.OnStart();
    DispatcherVisualizer visualizer(controller);
    visualizer.Run("Controller dispatcher");
    controller.OnStop();
    printf("Done.\n");
  }
  else {
    printf("Running controller dispatcher server loop...\n");
    RunForever(controller);
    printf("Done.\n");
  }

  printf("Shutting down controller dispatcher...\n");
  //kill the python bindings
  UnregisterPyTaskControllers();
  Py_Finalize();
  sendShutdown();
  return 0;
}

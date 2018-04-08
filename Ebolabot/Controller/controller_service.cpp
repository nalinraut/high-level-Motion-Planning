/** This program provides an operator assistance controller for
 * 1) reading in EbolabotTask messages and managing the low-level controller
 * 2) commanding the Ebolabot via the Klamp't controller communication protocol (CCP),
 * 3) broadcasting the Ebolabot state from the Baxter to the system state service.
 *
 * RUNNING:
 * For physical robot mode:
 * 0. Run EbolabotUI/system_state_service.py (check the printout to make sure
 *    it's set up on localhost:4568).
 * 1. Start up the robot, connect it via ROS (~/ros_ws/.baxter_setup.sh), and enable
 *    its motors (rosrun baxter_tools enable_robot.py -e).
 * 2. Run the Klampt->Baxter relay program
 *    (python baxterserialrelay.py -m k2b -r [PATH-TO-KLAMPT-ROBOT-MODEL]).
 * 3. Run this program.
 * 4. Run your UI (example: python EbolabotUI/haptic_widget.py) to start serving
 *    tasks to this program.
 *
 * For simulated robot mode:
 * 0. Run EbolabotUI/system_state_service.py (check the printout to make sure
 *    it's set up on localhost:4568),
 * 1. Run SimTest and open up a serial controller on localhost:3456.
 * 2. Run this program.
 * 3. Run your UI (example: python EbolabotUI/haptic_widget.py) to start serving
 *    tasks to this program.
 *
 * Published state topics
 * - .robot.t: time
 *         .dt: Klamp't controller command time step
 *         .sensors.q: sensed joint configuration
 *         .sensors.dq: sensed joint velocity
 *         .sensors.?: any other sensor data sent by the klampt rosbaxtercontroller.py script
 *         .command.qcmd: commanded joint configuration
 *         .command.dqcmd: commanded joint configuration
 *         .endEffectors[0,1].xform: current end effector transform
 *                           .dest_xform: endpoint end effector transform
 * - .controller.traj_t_end: the end time of the queued motion
 *              .traj_q_end: the end configuation of the queued motion
 *              .traj.times: the trajectory milestone times of the queued motion (20 / s)
 *              .traj.milestones: the trajectory milestones of the queued motion (20 / s)
 *
 * Subscribed state topics
 * - .controller.task: the requested task
 *
 * Implemented tasks:
 * - .type = move_to_joint
     .target = qdes
 * - .type = multi_ik
 *   .components = [ikgoal1, ikgoal2, ...]
 */
#include "Planning/RealTimePlanner.h"
#include "Planning/RealTimeIKPlanner.h"
#include "Interface/RobotInterface.h"
#include "Control/SerialControlledRobot.h"
#include "IO/XmlWorld.h"
#include "IO/JSON.h"
#include <KrisLibrary/utils/stringutils.h>
#include <sspp/Topic.h>
#include <sspp/Send.h>
#include <fstream>

//planner update time step
double dt=0.01;
//coefficient for the path duration cost in RealTimePlannerBase
double timeCostCoeff = 0.0;

//serial communication port to Klamp't -> Baxter (ROS) bridge
//to launch the bridge run "rosrun klampt baxterserialrelay.py -m k2b"
int commsPort = 3456;

//serial communication address from Klamp't <- ebolabot state server
const char* stateServer = "tcp://localhost:4568";



/** A hook for making a controller broadcast its command/sensor
 * data to the system state service every time it is updated
 */
class MotionQueueBroadcastController : public PolynomialPathController
{
public:
  File systemStateService;
  MotionQueueInterface* queue;

  MotionQueueBroadcastController(Robot& robot,const char* stateServer,MotionQueueInterface* _queue)
    :PolynomialPathController(robot),queue(_queue)
  {
    Assert(!systemStateService.IsOpen());
  }

  virtual ~MotionQueueBroadcastController() { }

  virtual void Update(Real dt) {
    PolynomialPathController::Update(dt);

    if(time == 0) return;

    if(!systemStateService.IsOpen()) {
      printf("MotionQueueBroadcast connecting to state server...\n");
      if(!systemStateService.Open(stateServer,FILECLIENT)) {
	FatalError("Could not open socket to system state service");
      }
    }

    //provide feedback about the time
    {
      AnyCollection data;
      data["t"] = time;
      data["dt"] = 1.0/dt;
      AnyCollection command;
      command["type"] = string("change");
      command["path"] = string(".robot");
      command["data"] = data;
      bool res=SSPP::Send(systemStateService,command);
      Assert(res);
    }

    //provide feedback about the command
    bool isPID = true;
    for(size_t i=0;i<command->actuators.size();i++) {
      if(command->actuators[i].mode != ActuatorCommand::PID)
	isPID = false;
    }
    if(isPID) {
      Config qcmd,dqcmd;
      GetCommandedConfig(qcmd);
      GetCommandedVelocity(dqcmd);
      AnyCollection commandData;
      commandData["qcmd"] = vector<double>(qcmd);
      commandData["dqcmd"] = vector<double>(dqcmd);
      AnyCollection command;
      command["type"] = string("set");
      command["path"] = string(".robot.command");
      command["data"] = commandData;
      bool res=SSPP::Send(systemStateService,command);
      Assert(res);
    }

    {
      //provide feedback about the sensors
      AnyCollection sensorData;
      for(size_t i=0;i<sensors->sensors.size();i++) {
	vector<double> values;
	sensors->sensors[i]->GetMeasurements(values);
	sensorData[sensors->sensors[i]->name] = AnyCollection(values);
      }
      AnyCollection command;
      command["type"] = string("set");
      command["path"] = string(".robot.sensors");
      command["data"] = sensorData;
      bool res=SSPP::Send(systemStateService,command);
      Assert(res);
    }

    if (queue) {
      //provide feedback about the motion queue
      AnyCollection command;
      command["type"] = string("change");
      command["path"] = string(".controller");
      AnyCollection data;
      double tstart = queue->GetCurTime();
      double tend = queue->GetEndTime();
      data["traj_t_end"] = tend;
      //read out the path?
      Config q;
      Real dt = 0.05;
      int istart=(int)Ceil(tstart/dt);
      int iend=(int)Ceil(tend/dt);
      AnyCollection path;
      vector<double> times;
      for(int i=istart;i<iend;i++) {
	Real t=i*dt;
	times.push_back(t);
	queue->GetConfig(t,q);
	path[i-istart] = vector<double>(q);
      }
      queue->GetEndConfig(q);
      path[iend-istart] = vector<double>(q);
      times.push_back(tend);
      /*
      data["traj"]["milestones"] = path;
      data["traj"]["times"] = times;
      */
      data["traj_q_end"] = vector<double>(q);
      command["data"] = data;
      bool res=SSPP::Send(systemStateService,command);
      Assert(res);
    }
    else printf("queue does not exist??\n");

    //provide feedback about the end effectors
    {
      Config qcmd;
      GetCommandedConfig(qcmd);
      robot.UpdateConfig(qcmd);
      const static int eeIndices[2] = {25,45};
      AnyCollection data;
      data.resize(2);
      for(int i=0;i<2;i++) {
	const RigidTransform& T = robot.links[eeIndices[i]].T_World;
	//convert transform to object
	vector<double> R(9);
	vector<double> t(3);
	T.R.get(&R[0]);
	T.t.get(&t[0]);
	data[i]["xform"].resize(2);
	data[i]["xform"][0] = R;
	data[i]["xform"][1] = t;
      }
      if(queue) {
	queue->GetEndConfig(qcmd);
	robot.UpdateConfig(qcmd);
	for(int i=0;i<2;i++) {
	  const RigidTransform& T = robot.links[eeIndices[i]].T_World;
	  //convert transform to object
	  vector<double> R(9);
	  vector<double> t(3);
	  T.R.get(&R[0]);
	  T.t.get(&t[0]);
	  data[i]["dest_xform"].resize(2);
	  data[i]["dest_xform"][0] = R;
	  data[i]["dest_xform"][1] = t;
	}
      }
      AnyCollection command;
      command["type"] = string("set");
      command["path"] = string(".robot.endEffectors");
      command["data"] = data;
      bool res=SSPP::Send(systemStateService,command);
      Assert(res);
    }
  }
};



/** A thread that serves a controller to a Klamp't serial robot
 * on the given address.
 */
class RobotCommunicationThread
{
public:
  RobotCommunicationThread(const char* addr="tcp://localhost:3456");
  virtual ~RobotCommunicationThread();
  void Start(RobotController* server);
  void Stop();
  //calling thread needs to call this before working on the queue
  void Lock();
  //calling thread needs to call this after working on the queue
  void Unlock();

  void* internal;
  Thread thread;
};


struct RobotCommunicationThreadData
{
  string addr;   //(in)
  RobotController* controller;     //(in)
  bool ready;  //(out)
  SerialControlledRobot* comms; //(out): can be used to stop robot
  Mutex mutex;  //mutex is used inside comms object to control access to motion queue and controller
};

void* communicationThreadFunc(void* vdata)
{
  RobotCommunicationThreadData* data = (RobotCommunicationThreadData*)vdata;
  data->ready = false;
  printf("SerialControlledRobot connecting to %s\n",data->addr.c_str());
  SerialControlledRobot comms(data->addr.c_str());
  comms.SetMutex(&data->mutex);
  data->comms = &comms;
  comms.Init(&data->controller->robot,data->controller);
  printf("SerialControlledRobot on %s initialized\n",data->addr.c_str());
  if(comms.Process(Inf)) {
    //run loop
    data->ready = true;
    comms.Run();
    //stopped
    data->ready = false;
  }
  else {
    printf("Robot communication thread: error processing first message\n");
  }
  return data;
}

RobotCommunicationThread::RobotCommunicationThread(const char* addr)
{
  internal = new RobotCommunicationThreadData;
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  data->addr = addr;
}

RobotCommunicationThread::~RobotCommunicationThread()
{
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  delete data;
}

void RobotCommunicationThread::Lock()
{
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  data->mutex.lock();
}

void RobotCommunicationThread::Unlock()
{
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  data->mutex.unlock();
}

void RobotCommunicationThread::Start(RobotController* controller)
{
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  data->ready = false;
  data->controller = controller;
  thread = ThreadStart(communicationThreadFunc,data);
  int cnt = 0;
  while(!data->ready) {
    if(cnt % 10 == 0)
      printf("Motion queue thread, waiting for first communication with robot\n");
    ThreadSleep(0.1);
    cnt++;
  }
}

void RobotCommunicationThread::Stop()
{
  RobotCommunicationThreadData* data = reinterpret_cast<RobotCommunicationThreadData*>(internal);
  data->comms->Stop();
  while(data->ready) {
    ThreadSleep(0.1);
  }
  ThreadJoin(thread);
}

/** Usage
 * EbolaBotController c();
 * c.Init();
 * c.Run();
 * In an external thread you may call c.Terminate();
 */
class EbolabotController : public SSPP::TopicServiceBase
{
public:
  RobotWorld planningWorld;
  WorldPlannerSettings settings;
  string initialState;

  bool connected;
  RobotCommunicationThread communicationThread;
  RealTimePlanningThread planningThread;
  enum PlannerType { None, IKPlanner };
  PlannerType currentPlanner;

  SmartPointer<MotionQueueBroadcastController> controller;
  SmartPointer<DefaultMotionQueueInterface> robotInterface;

  float collisionMargin,oldCollisionMargin;
  SmartPointer<SingleRobotCSpace> cspace;

  EbolabotController()
    :SSPP::TopicServiceBase(stateServer,".controller.task")
  { 
    SSPP::TopicServiceBase::tolerateReadErrors = true;
  }

  ~EbolabotController() {
    communicationThread.Stop();
    planningThread.Stop();
  }

  virtual const char* Name() const { return "EbolabotController"; }

  virtual const char* Description() const { return "Runs Ebolabot planning / motion queue"; }

  virtual int Process() {
    int n=SSPP::TopicServiceBase::Process();
    if(n < 0) return n;
    communicationThread.Lock();
    planningThread.SendUpdate(robotInterface);
    communicationThread.Unlock();
    return n;
  }

  virtual bool OnMessage(AnyCollection& message) { 
    if(!connected) return false;  //quit

    //it's a new task! parse it and set up the correct cspace
    string type;
    if(!message["type"].as(type)) {
      printf("Message does not contain type attribute\n");
      cout<<message<<endl;
      return false;
    }
    if(type == "multi_ik") {
      size_t n=message["components"].size();
      if(n == 0) {
	printf("Got an empty multi-ik objective, stopping planning\n");
	planningThread.SetObjective(NULL);
      }
      else {
	vector<IKGoal> goals(n);
	for(size_t i=0;i<n;i++) {
	  if(!Convert(message["components"][(int)i],goals[i])) {
	    printf("Error converting IK constraint %d\n",(int)i);
	    planningThread.SetObjective(NULL);
	    return true;
	  }
	}
	printf("Setting objective with %d IK constraints\n",(int)n);

	CompositeObjective* obj = new CompositeObjective;
	for(size_t i=0;i<n;i++) {
	  IKObjective* ikobj = new IKObjective(planningWorld.robots[0].robot);
	  ikobj->ikGoal = goals[i];
	  obj->Add(ikobj);
	}
	if(currentPlanner != IKPlanner) {
	  planningThread.SetPlanner(new DynamicIKPlanner);
	  planningThread.SetCSpace(cspace);
	  currentPlanner = IKPlanner;
	}
	//the thread takes ownership of the pointer
	planningThread.SetObjective(obj);
      }
    }
    else if(type == "move_to_joint") {
      printf("Got a move_to_joint task\n");
    }
    else {
      printf("Got an unknown task %s\n",type.c_str());
      planningThread.SetObjective(NULL);
    }
    return true; 
  }
  
  void Init(RobotWorld& world) {
    settings.InitializeDefault(world);
    collisionMargin = oldCollisionMargin = 0;
    currentPlanner = None;

    controller = new MotionQueueBroadcastController(*world.robots[0].robot,stateServer,NULL);
    robotInterface = new DefaultMotionQueueInterface(controller);
    controller->queue = robotInterface;
    connected = false;
    CopyWorld(world,planningWorld);
    Robot* robot = planningWorld.robots[0].robot;
    for(size_t i=0;i<robot->geometry.size();i++) {
      robot->geometry[i].margin += collisionMargin;
    }

    cspace = new SingleRobotCSpace(planningWorld,0,&settings);

    while(!connected) {
      /*
      printf("Ebolabot controller connecting to state server %s\n",stateServer);
      if(!SSPP::TopicServiceBase::OpenClient(stateServer)) {
	fprintf(stderr,"Couldn't connect to state server, waiting...\n");
	ThreadSleep(1.0);
	continue;
      }
      */
      printf("Subscribing to %s\n",topic.c_str());
      SSPP::TopicServiceBase::Subscribe();

      printf("Starting communication thread...\n");

      //connect and start execution thread
      communicationThread.Start(controller);

      connected = true;
    }

    //initialize basic planning info
    printf("Init planner...\n");
    planningThread.SetPlanner(new RealTimePlanner);
    bool gotStart = false;
    vector<double> q;
    while(!gotStart) {
      printf("Reading from .robot.command.qcmd from state server...\n");
      AnyCollection msg = SSPP::ReadTopic(stateServer,".robot.command.qcmd");
      if(!msg.asvector(q)) {
	cout<<"Got "<<msg<<endl;
	printf("Are you sure you set up the state server?\n");
	printf("Waiting...\n");
	ThreadSleep(1.0);
      }
      else
	gotStart = true;
    }
    cout<<"Planner start configuration: "<<Config(q)<<endl;
    planningThread.SetStartConfig(q);

    printf("Connected! ready to Run!\n");
  }

  void Run() {
    printf("Starting planning thread...\n");

    planningThread.Start();
    printf("Running forever...\n");
    SSPP::RunForever(*this);
  }

  void Terminate() {
    planningThread.Stop();
    communicationThread.Stop();
    connected = false;
  }

  void SetCollisionMargin(double collisionMargin) {
    Robot* robot = planningWorld.robots[0].robot;
    for(size_t i=0;i<robot->geometry.size();i++)
      robot->geometry[i].margin -= oldCollisionMargin;
    for(size_t i=0;i<robot->geometry.size();i++)
      robot->geometry[i].margin += collisionMargin;
    oldCollisionMargin = collisionMargin;
  }
};


int main(int argc, char** argv)
{
  if(argc < 2) {
    printf("USAGE: ControllerService XML_file\n");
    return 0;
  }
  RobotWorld world;
  XmlWorld xmlWorld;
  for(int i=1;i<argc;i++) {
    const char* ext=FileExtension(argv[i]);
    if(0==strcmp(ext,"rob")) {
      if(world.LoadRobot(argv[i])<0) {
	printf("Error loading robot file %s\n",argv[i]);
	return 1;
      }
    }
    else if(0==strcmp(ext,"env") || 0==strcmp(ext,"tri")) {
      if(world.LoadTerrain(argv[i])<0) {
	printf("Error loading terrain file %s\n",argv[i]);
	return 1;
      }
    }
    else if(0==strcmp(ext,"obj")) {
      if(world.LoadRigidObject(argv[i])<0) {
	printf("Error loading rigid object file %s\n",argv[i]);
	return 1;
      }
    }
    else if(0==strcmp(ext,"xml")) {
      if(!xmlWorld.Load(argv[i])) {
	printf("Error loading world file %s\n",argv[i]);
	return 1;
      }
      if(!xmlWorld.GetWorld(world)) {
	printf("Error loading world from %s\n",argv[i]);
	return 1;
      }
    }
    else {
      printf("Unknown file extension %s on file %s\n",ext,argv[i]);
      return 1;
    }
  }

  EbolabotController controller;
  controller.Init(world);
  controller.Run();
  return 0;
}

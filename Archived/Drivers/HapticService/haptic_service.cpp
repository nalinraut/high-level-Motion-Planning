#ifdef  _WIN64 
#pragma warning (disable:4996)
#endif

#ifdef _WIN32
#pragma warning (disable:4996)
#endif

#include <Windows.h>
#include <cstdio>
#include <cassert>
#include <iostream>
#include <utils/threadutils.h>
#include <Timer.h>
#include <math3d/primitives.h>
#include <math3d/rotation.h>
#include <utils/AsyncIO.h>
#include <utils/AnyCollection.h>

#if defined(WIN32)
# include <conio.h>
#else
# include "conio.h"
#endif

#include <HD/hd.h>
using namespace std;

static bool server = true;
static string host = "tcp://localhost:4567";
static double writeRate = 50.0;

const char *getErrorCodeName(HDerror errorCode)
{
    switch (errorCode)
    {
        case HD_SUCCESS: return "HD_SUCCESS";

        /* Function errors */
        case HD_INVALID_ENUM: return "HD_INVALID_ENUM";
        case HD_INVALID_VALUE: return "HD_INVALID_VALUE";
        case HD_INVALID_OPERATION: return "HD_INVALID_OPERATION";
        case HD_INVALID_INPUT_TYPE: return "HD_INVALID_INPUT_TYPE";
        case HD_BAD_HANDLE: return "HD_BAD_HANDLE";

        /* Force errors */
        case HD_WARM_MOTORS: return "HD_WARM_MOTORS";
        case HD_EXCEEDED_MAX_FORCE: return "HD_EXCEEDED_MAX_FORCE";
        case HD_EXCEEDED_MAX_VELOCITY: return "HD_EXCEEDED_MAX_VELOCITY";
        case HD_FORCE_ERROR: return "HD_FORCE_ERROR";

        /* Device errors */
        case HD_DEVICE_FAULT: return "HD_DEVICE_FAULT";
        case HD_DEVICE_ALREADY_INITIATED: return "HD_DEVICE_ALREADY_INITIATED";
        case HD_COMM_ERROR: return "HD_COMM_ERROR";
        case HD_COMM_CONFIG_ERROR: return "HD_COMM_CONFIG_ERROR";
        case HD_TIMER_ERROR: return "HD_TIMER_ERROR";

        /* Haptic rendering context */
        case HD_ILLEGAL_BEGIN: return "HD_ILLEGAL_BEGIN";
        case HD_ILLEGAL_END: return "HD_ILLEGAL_END";
        case HD_FRAME_ERROR: return "HD_FRAME_ERROR";

        /* Scheduler errors */
        case HD_INVALID_PRIORITY: return "HD_INVALID_PRIORITY";
        case HD_SCHEDULER_FULL: return "HD_SCHEDULER_FULL";

        /* Licensing errors */
        case HD_INVALID_LICENSE: return "HD_INVALID_LICENSE";

        default: return "Unknown Error Code";
    }
}


/******************************************************************************
 Pretty prints a message containing the error code name, the error code and
 the internal error code.
******************************************************************************/
void hduPrintError(FILE *stream, const HDErrorInfo *error,
                   const char *message)
{
    fprintf(stream, "HD Error: %s\n", getErrorCodeName(error->errorCode));
    fprintf(stream, "%s\n", hdGetErrorString(error->errorCode));
    fprintf(stream, "HHD: %X\n", error->hHD);
    fprintf(stream, "Error Code: %X\n", error->errorCode);
    fprintf(stream, "Internal Error Code: %d\n", error->internalErrorCode);
    fprintf(stream, "Message: %s\n", message);
}

/******************************************************************************
 Pretty prints a message containing the error code name, the error code and
 the internal error code.
******************************************************************************/
ostream &operator<<(ostream &os, const HDErrorInfo &error)
{    
    os << "HD Error : " << getErrorCodeName(error.errorCode) << endl;
    os << hdGetErrorString(error.errorCode) << endl;
	os.setf(ios::hex);
    os << "HHD: " << error.hHD << endl;
    os << "Error Code: " << error.errorCode << endl;
	os.setf(ios::dec);
    os << "Internal Error Code: " << error.internalErrorCode << endl;

    return os;    
}

/******************************************************************************
 Indicates whether this is a recoverable error related to haptic rendering.
******************************************************************************/
HDboolean hduIsForceError(const HDErrorInfo *error)
{
    switch (error->errorCode)
    {
        case HD_WARM_MOTORS:   
        case HD_EXCEEDED_MAX_FORCE:
        case HD_EXCEEDED_MAX_VELOCITY:
        case HD_FORCE_ERROR:
            return HD_TRUE;

        default:
            return HD_FALSE;
    }
}

/******************************************************************************
 Indicates whether this is a communication, timing or general scheduler error
 that would prevent proper operation of the servo loop
******************************************************************************/
HDboolean hduIsSchedulerError(const HDErrorInfo *error)
{
    switch (error->errorCode)
    {
        case HD_COMM_ERROR:
        case HD_COMM_CONFIG_ERROR:
        case HD_TIMER_ERROR:
        case HD_INVALID_PRIORITY:
        case HD_SCHEDULER_FULL:
            return HD_TRUE;

        default:
            return HD_FALSE;
    }
}


#ifdef WIN32
void clrscr()
{
	system("cls");
}
#endif //WIN32

//Data for a single device
struct DeviceCallbackData
{
	DeviceCallbackData();

	HHD handle;

	//out
	int buttons;         //button state of stylus
	int inkwell;        //inkwell state
	double position[3];  //cartesian position of stylus
	double rotationMoment[3];   //rotation matrix of stylus, in MomentRotation form
	double velocity[3];  //cartesian velocity of stylus
	double angularVelocity[3];
	double jointAngle[3];	// joint angle of device
	double gimbalAngle[3];	// joint angle of gimbal
	double force[3];    //force applied by force feedback setting 
	double lastTime;    //last update time
	int iteration;

	//in: force feedback linear term: f = forceLinear*(p-forceCenter)+forceDamping*(v-forceVelocityCenter)+forceConstant, where p and v are the stylus position/velocity
	bool applyForce,overrideContinuousForceLimits;
	double forceCenter[3],forceVelocityCenter[3];
	double forceLinear[9],forceDamping[9];
	double forceConstant[3];

	//properties
	double maxStiffness,maxDamping;
	double maxForce,maxContinuousForce;
};

//extracted data for all devices
struct CallbackData
{
	CallbackData();

	vector<DeviceCallbackData> devices;
	Mutex mutex;
	Timer timer;
};

DeviceCallbackData::DeviceCallbackData()
	:buttons(0),inkwell(0),lastTime(0),iteration(0),applyForce(0),overrideContinuousForceLimits(0),maxStiffness(0),maxDamping(0),maxForce(0),maxContinuousForce(0)
{
	fill(position,position+3,0.0);
	fill(rotationMoment,rotationMoment+3,0.0);
	fill(velocity,velocity+3,0.0);
	fill(jointAngle,jointAngle+3,0.0);
	fill(gimbalAngle,gimbalAngle+3,0.0);
	fill(force,force+3,0.0);
	fill(forceCenter,forceCenter+3,0.0);
	fill(forceVelocityCenter,forceVelocityCenter+3,0.0);
	fill(forceLinear,forceLinear+9,0.0);
	fill(forceDamping,forceDamping+9,0.0);
	fill(forceConstant,forceConstant+3,0.0);
}

CallbackData::CallbackData()
{}

/*******************************************************************************
My callback: read into CallbackData object
*******************************************************************************/
HDCallbackCode HDCALLBACK PrintCallback(void *dataPtr)
{
	CallbackData* cbdata = static_cast<CallbackData*>(dataPtr);
	cbdata->mutex.lock();
	double time = cbdata->timer.ElapsedTime();

	for(size_t i=0;i<cbdata->devices.size();i++) {
		hdMakeCurrentDevice(cbdata->devices[i].handle);
		hdBeginFrame(cbdata->devices[i].handle);
		DeviceCallbackData* data = &cbdata->devices[i];
		if(data->iteration==0) {
			hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS,&data->maxStiffness);
			hdGetDoublev(HD_NOMINAL_MAX_DAMPING,&data->maxDamping);
			hdGetDoublev(HD_NOMINAL_MAX_FORCE,&data->maxForce);
			hdGetDoublev(HD_NOMINAL_MAX_CONTINUOUS_FORCE,&data->maxContinuousForce);
		}

		// Get the position of the device.
		double transform[16];
		hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
		hdGetIntegerv(HD_CURRENT_BUTTONS,&data->buttons);
		//hdGetIntegerv(HD_CURRENT_SAFETY_SWITCH,&data->inkwell);
		hdGetDoublev(HD_CURRENT_VELOCITY,data->velocity);
		hdGetDoublev(HD_CURRENT_ANGULAR_VELOCITY,data->angularVelocity);
		hdGetDoublev(HD_CURRENT_JOINT_ANGLES,data->jointAngle);
		hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,data->gimbalAngle);

		Math3D::Matrix4 mat(transform);
		Math3D::RigidTransform T(mat);
		//convert from mm to m
		T.t *= 1e-3;
		T.t.get(data->position);
		data->velocity[0] *= 1e-3;
		data->velocity[1] *= 1e-3;
		data->velocity[2] *= 1e-3;

		//get rotation
		Math3D::MomentRotation m;
		m.setMatrix(T.R);
		m.get(data->rotationMoment);

		//update timestamp
		static double oldTime = 0;
		static int oldIteration = 0;
		static double oldRate;
		double dt = time - data->lastTime;
		data->lastTime = time;
		data->iteration += 1;

		//for printing later outside of lock
		int buttons = data->buttons;
		double rate = oldRate;

		//update rate printer
		if(data->iteration % 1000 == 0) {
			oldRate = (data->iteration-oldIteration)/(data->lastTime - oldTime);
			oldTime = data->lastTime;
			oldIteration = data->iteration;
		}


		//this is how to set force feedback
		if(data->applyForce) {
			Math3D::Vector3 f = Math3D::Matrix3(data->forceLinear)*(T.t-Math3D::Vector3(data->forceCenter)) + Math3D::Vector3(data->forceConstant);
			f += Math3D::Matrix3(data->forceDamping)*(Math3D::Vector3(data->velocity)-Math3D::Vector3(data->forceVelocityCenter));
			//integrate force center
			Math3D::Vector3 newcenter = Math3D::Vector3(data->forceCenter) + Math3D::Vector3(data->forceVelocityCenter)*dt;
			newcenter.get(data->forceCenter);
			double fmax = data->maxContinuousForce;
			if(data->overrideContinuousForceLimits)
				fmax = data->maxForce;
			f.x = Math::Clamp(f.x,-fmax,fmax);
			f.y = Math::Clamp(f.y,-fmax,fmax);
			f.z = Math::Clamp(f.z,-fmax,fmax);
			f.get(data->force);
			hdSetDoublev(HD_CURRENT_FORCE, f);
		}
		else {
			bool set = (data->force[0] != 0 || data->force[1] != 0 || data->force[2] != 0);
			data->force[0] = data->force[1] = data->force[2] = 0.0;
			if(set)
				hdSetDoublev(HD_CURRENT_FORCE, data->force);
		}
		hdEndFrame(cbdata->devices[i].handle);

		//printf("buttons: %d %d %d %d\n",(buttons & HD_DEVICE_BUTTON_1 ? 1:0),(buttons & HD_DEVICE_BUTTON_2? 1:0),(buttons & HD_DEVICE_BUTTON_3 ? 1:0),(buttons & HD_DEVICE_BUTTON_4 ? 1:0));
		//cout<<"t: "<<T.t<<", r: "<<m<<endl;
		if(data->iteration % 1000 == 0 && i == 0) {
			printf("Rate: %g\n",oldRate);
			printf("Press any key to quit.\n\n");
		}
	}
	cbdata->mutex.unlock();

	for(size_t i=0;i<cbdata->devices.size();i++) {
		hdMakeCurrentDevice(cbdata->devices[i].handle);
		HDErrorInfo error;
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Error during main scheduler callback\n");

			if (hduIsSchedulerError(&error))
			{
				return HD_CALLBACK_DONE;
			}        
		}
	}
    return HD_CALLBACK_CONTINUE;
}

bool ParseCommand(AnyCollection& msg,CallbackData& data)
{
	string type;
	if(!msg["type"].as(type)) {
		fprintf(stderr,"ParseCommand: input message doesn't have type member\n");
		cout<<"Message: "<<msg<<endl;
		return false;
	}
	if(type != "HapticForceCommand" && type != "MultiHapticForceCommand") {
		fprintf(stderr,"ParseCommand: input message must be of type HapticForceCommand or MultiHapticForceCommand\n");
		return false;
	}
	if(type == "HapticForceCommand") {
		int device=0;
		bool enabled,overrideContinuousForceLimits;
		vector<double> center,linear,constant,damping,velocityCenter;
		if(!msg["device"].as(device) && data.devices.size() > 1) {
			fprintf(stderr,"ParseCommand: input message must specify the device index\n");
			return false;
		}
		if(device < 0 || device >= (int)data.devices.size()) {
			fprintf(stderr,"ParseCommand: input message has invalid device index %d (maximum: %d)\n",device,data.devices.size()-1);
			return false;
		}
		if(!msg["enabled"].as(enabled)) {
			fprintf(stderr,"ParseCommand: input message missing \"enabled\" field\n");
			return false;
		}
		if(!msg["overrideContinuousForceLimits"].as(overrideContinuousForceLimits)) {
			overrideContinuousForceLimits = false;
		}
		if(!msg["center"].asvector(center)) {
			center.resize(3,0.0);
		}
		if(center.size() != 3) {
			fprintf(stderr,"ParseCommand: invalid size of \"center\" field\n");
			return false;
		}
		if(!msg["constant"].asvector(constant)) {
			constant.resize(3,0.0);
		}
		if(center.size() != 3) {
			fprintf(stderr,"ParseCommand: invalid size of \"constant\" field\n");
			return false;
		}
		if(!msg["velocityCenter"].asvector(velocityCenter)) {
			velocityCenter.resize(3,0.0);
		}
		if(velocityCenter.size() != 3) {
			fprintf(stderr,"ParseCommand: invalid size of \"velocityCenter\" field\n");
			return false;
		}
		
		
		//read 1, 3, or 9 entries
		double clinear;
		if(!msg["linear"].asvector(linear)) {
			linear.resize(9,0.0);
			//a single scalar entry
			if(msg["linear"].as(clinear)) {
				linear[0] = linear[4] = linear[8] = clinear;
			}
		}
		if(linear.size() != 1 && linear.size() != 3 && linear.size() != 9) {
			fprintf(stderr,"ParseCommand: invalid size of \"linear\" field\n");
			return false;
		}
		if(linear.size()==1) {
			double clinear = linear[0];
			linear.resize(9,0.0);
			linear[0] = linear[4] = linear[8] = clinear;
		}
		if(linear.size()==3) {
			vector<double> cp = linear;
			linear.resize(9);
			fill(linear.begin(),linear.end(),0);
			linear[0] = cp[0];
			linear[4] = cp[1];
			linear[8] = cp[2];
		}
		//read 1, 3, or 9 entries
		double cdamping;
		if(!msg["damping"].asvector(damping)) {
			damping.resize(9,0.0);
			//a single scalar entry
			if(msg["damping"].as(cdamping)) {
				damping[0] = damping[4] = damping[8] = cdamping;
			}
		}
		if(damping.size() != 1 && damping.size() != 3 && damping.size() != 9) {
			fprintf(stderr,"ParseCommand: invalid size of \"damping\" field\n");
			return false;
		}
		if(damping.size()==1) {
			double cdamping = damping[0];
			damping.resize(9,0.0);
			damping[0] = damping[4] = damping[8] = cdamping;
		}
		if(damping.size()==3) {
			vector<double> cp = damping;
			damping.resize(9);
			fill(damping.begin(),damping.end(),0);
			damping[0] = cp[0];
			damping[4] = cp[1];
			damping[8] = cp[2];
		}
		data.mutex.lock();
		data.devices[device].applyForce = enabled;
		data.devices[device].overrideContinuousForceLimits = overrideContinuousForceLimits;
		copy(center.begin(),center.end(),data.devices[device].forceCenter);
		copy(constant.begin(),constant.end(),data.devices[device].forceConstant);
		copy(velocityCenter.begin(),velocityCenter.end(),data.devices[device].forceVelocityCenter);
		copy(linear.begin(),linear.end(),data.devices[device].forceLinear);
		copy(damping.begin(),damping.end(),data.devices[device].forceDamping);
		data.mutex.unlock();
	}
	else {
		fprintf(stderr,"Can't do MultiHapticCommand yet\n");
		return false;
	}
	return true;
}

struct DeviceEvent
{
	enum Type { InkwellOn, InkwellOff, Button1Down, Button1Up, Button2Down, Button2Up };
	Type type;
	double time;
};

class HapticService : public SocketPipeWorker
{
public:
	double nextSendTime;
	vector<DeviceCallbackData> lastData;
	vector<vector<DeviceEvent> > events;

	HapticService(const char* addr,bool asServer=false,double timeout=Math::Inf)
		:SocketPipeWorker(addr,asServer,timeout),nextSendTime(0)
	{}
	int Process(CallbackData& data)
	{
		//read new force messages
		if(this->NewMessageCount() > 0) {
			string str = this->NewestMessage();
			if(!str.empty())  {
				stringstream ss(str);
				AnyCollection msg;
				ss>>msg;
				if(!ss) {
					fprintf(stderr,"ProcessMessages: Got an improperly formatted string\n");
					return -1;
				}
				ParseCommand(msg,data);
			}
			else {
				printf("Weird... read in empty string??\n");
				getchar();
			}
		}
		//check if the worker thread is in a waiting status
		if(!this->WriteReady()) {
			//disable force feedback
			data.mutex.lock();
			for(size_t i=0;i<data.devices.size();i++)
				data.devices[i].applyForce = false;
			data.mutex.unlock();
			//printf("Worker waiting...\n");
			return 0;
		}
		if(data.devices.empty()) return 0;
	
		//parse events
		data.mutex.lock();
		double time = data.timer.ElapsedTime();
		if(events.empty()) events.resize(data.devices.size());
		if(!lastData.empty()) {
			for(size_t i=0;i<data.devices.size();i++) {
				if(data.devices[i].inkwell && !lastData[i].inkwell) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::InkwellOn;
					events[i].back().time = time;
				}
				else if(!data.devices[i].inkwell && lastData[i].inkwell) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::InkwellOff;
					events[i].back().time = time;
				}
				if((data.devices[i].buttons & 0x1) && !(lastData[i].buttons & 0x1)) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::Button1Down;
					events[i].back().time = time;
				}
				else if(!(data.devices[i].buttons & 0x1) && (lastData[i].buttons & 0x1)) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::Button1Up;
					events[i].back().time = time;
				}
				if((data.devices[i].buttons & 0x2) && !(lastData[i].buttons & 0x2)) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::Button2Down;
					events[i].back().time = time;
				}
				else if(!(data.devices[i].buttons & 0x2) && (lastData[i].buttons & 0x2)) {
					events[i].resize(events[i].size()+1);
					events[i].back().type = DeviceEvent::Button2Up;
					events[i].back().time = time;
				}
			}
		}
		lastData = data.devices;
		//data.mutex.unlock();

		//write to socket
		//data.mutex.lock();
		//double time = data.timer.ElapsedTime();
		if(time >= this->nextSendTime) {
			this->nextSendTime += 1.0/writeRate;
			vector<DeviceCallbackData> devices = data.devices;
			data.mutex.unlock();
			//unlock mutex before forming string -- because it takes a nonnegligible amount of time
			AnyCollection msg;
			msg["type"] = string("MultiHapticState");
			for(size_t i=0;i<devices.size();i++) {
				DeviceCallbackData& ddata = devices[i];
				//write the data to the worker thread
				AnyCollection dmsg,position,velocity,angvel,rotation,force, jointAngle, gimbalAngle;
				position.resize(3);
				position[0] = ddata.position[0];
				position[1] = ddata.position[1];
				position[2] = ddata.position[2];
				velocity.resize(3);
				velocity[0] = ddata.velocity[0];
				velocity[1] = ddata.velocity[1];
				velocity[2] = ddata.velocity[2];
				angvel.resize(3);
				angvel[0] = ddata.angularVelocity[0];
				angvel[1] = ddata.angularVelocity[1];
				angvel[2] = ddata.angularVelocity[2];
				rotation.resize(3);
				rotation[0] = ddata.rotationMoment[0];
				rotation[1] = ddata.rotationMoment[1];
				rotation[2] = ddata.rotationMoment[2];
				force.resize(3);
				force[0] = ddata.force[0];
				force[1] = ddata.force[1];
				force[2] = ddata.force[2];
				jointAngle.resize(3);
				jointAngle[0] = ddata.jointAngle[0];
				jointAngle[1] = ddata.jointAngle[1];
				jointAngle[2] = ddata.jointAngle[2];
				gimbalAngle.resize(3);
				gimbalAngle[0] = ddata.gimbalAngle[0];
				gimbalAngle[1] = ddata.gimbalAngle[1];
				gimbalAngle[2] = ddata.gimbalAngle[2];
								
				int button1 = (ddata.buttons & 0x1 ? 1 : 0);
				int button2 = (ddata.buttons & 0x2 ? 1 : 0);
				double time = ddata.lastTime;
				//form string
				dmsg["time"] = time;
				dmsg["button1"] = button1;
				dmsg["button2"] = button2;
				dmsg["position"] = position;
				dmsg["velocity"] = velocity;
				dmsg["angularVelocity"] = angvel;
				dmsg["jointAngle"] = jointAngle;
				dmsg["gimbalAngle"] =gimbalAngle;
				dmsg["rotationMoment"] = rotation;
				dmsg["force"] = force;
				msg["devices"][int(i)] = dmsg;
			}
			//add events
			msg["events"].resize(0);
			int n=0;
			for(size_t i=0;i<this->events.size();i++) {
				for(size_t j=0;j<this->events[i].size();j++) {
					AnyCollection e;
					e["time"] = this->events[i][j].time;
					e["device"] = (int)i;
					switch(this->events[i][j].type) {
					case DeviceEvent::Button1Down:
						e["type"] = string("b1_down"); 
						break;
					case DeviceEvent::Button1Up:
						e["type"] = string("b1_up"); 
						break;
					case DeviceEvent::Button2Down:
						e["type"] = string("b2_down"); 
						break;
					case DeviceEvent::Button2Up:
						e["type"] = string("b2_up"); 
						break;
					case DeviceEvent::InkwellOff:
						e["type"] = string("iw_off"); 
						break;
					case DeviceEvent::InkwellOn:
						e["type"] = string("iw_on"); 
						break;
					default:
						fprintf(stderr,"Invalid event type???\n");
						continue;
					}
					msg["events"][n] = e;
					n++;
				}
			}
			this->events.clear();

			//send the message
			stringstream ss;
			ss<<msg;
			this->SendMessage(ss.str());
			return 1;
		}
		else {
			data.mutex.unlock();
			return 0;
		}
	}
};

const char* USAGE_STRING = "HapticService [options] [device1, device2, ...]:\n\
Reads from a haptic device and runs a JSON message node that publishes state and\n\
receives force feedback commands.\n";
const char* OPTIONS_STRING = "OPTIONS:\n\
 -h,--help: prints this help message.\n\
 -r,--rate rate: sets the state publishing rate, in Hz (default 50).\n\
 -s,--server address: hosts a server on the given address (default localhost:4567).\n\
 -c,--client address: connects to a server on the given address.\n";
/******************************************************************************
 main function
 Initializes the device, creates a callback to read values, terminates
 upon key press.
******************************************************************************/
int main(int argc,const char* argv[])
{
	int i;
	for(i=1;i<argc;i++) {
		if(argv[i][0]=='-') {
			if(0==strcmp(argv[i],"-h") || 0==strcmp(argv[i],"--help") ) {
				printf(USAGE_STRING);
				printf(OPTIONS_STRING);
				return 0;
			}
			else if(0==strcmp(argv[i],"-r") || 0==strcmp(argv[i],"--rate")) {
				writeRate = atof(argv[i+1]);
				i++;
			}
			else if(0==strcmp(argv[i],"-s") ||0==strcmp(argv[i],"--server")) {
				server = true;
				host = argv[i+1];
				i++;
			}
			else if(0==strcmp(argv[i],"-c") ||0==strcmp(argv[i],"--client")) {
				server = false;
				host = argv[i+1];
				i++;
			}
			else {
				printf(OPTIONS_STRING);
				return 1;
			}
		}
		else break;
	}
	if(host.find("tcp://") != 0)
		host = "tcp://"+host;

	CallbackData data;
    HDErrorInfo error;
	if(i == argc) {
		// Initialize the default haptic device.
		HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
		if (HD_DEVICE_ERROR(error = hdGetError()))
		{
			hduPrintError(stderr, &error, "Failed to initialize haptic device");
			fprintf(stderr,"\n");
			fprintf(stderr,"Did you run the Geomagic Touch Setup program to pair the haptic device(s)?\n");
			fprintf(stderr, "\nPress any key to quit.\n");
			getch();
			return -1;
		}

		// Enable forces for the device.
		hdEnable(HD_FORCE_OUTPUT);

		data.devices.resize(data.devices.size()+1);
		data.devices.back().handle = hHD;
	}
	else {
		//initialize devices from command line
		while(i<argc) {
			// Initialize the default haptic device.
			HHD hHD = hdInitDevice(argv[i]);
			if (HD_DEVICE_ERROR(error = hdGetError()))
			{
				fprintf(stderr, "Error initializing haptic device \"%s\"\n",argv[i]);
				hduPrintError(stderr, &error, "Failed to initialize haptic device");
				fprintf(stderr,"\n");
				fprintf(stderr,"Did you run the Geomagic Touch Setup program to pair the haptic device(s)?\n");
				fprintf(stderr, "\nPress any key to quit.\n");
				getch();
				return -1;
			}

			// Enable forces for the device.
			hdEnable(HD_FORCE_OUTPUT);
			data.devices.resize(data.devices.size()+1);
			data.devices.back().handle = hHD;

			i++;
		}
	}

	//Start the servo scheduler 
    hdStartScheduler();
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
		fprintf(stderr,"\n");
		fprintf(stderr,"Did you run the Geomagic Touch Setup program to pair the haptic device(s)?\n");
		fprintf(stderr, "\nPress any key to quit.\n");
        getch();
		for(size_t i=0;i<data.devices.size();i++)
			hdDisableDevice(data.devices[i].handle);
        return -1;
    }
 
	printf("Starting socket on %s, writing at rate %g\n",host.c_str(),writeRate);
	HapticService worker(host.c_str(),server);
	//start worker thread
	if(!worker.Start()) {
		fprintf(stderr,"JSON service publisher failed to start...\n");
		for(size_t i=0;i<data.devices.size();i++)
			hdDisableDevice(data.devices[i].handle);
		return -1;
	}

    // Application loop - schedule our call to the main callback.
    HDSchedulerHandle hMyCallback = hdScheduleAsynchronous(
        PrintCallback, &data, HD_DEFAULT_SCHEDULER_PRIORITY);

    while (!_kbhit())
    {
        if (!hdWaitForCompletion(hMyCallback, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            break;
        }
		worker.Process(data);
    }

	worker.Stop();

    // For cleanup, unschedule our callbacks and stop the servo loop.
    hdStopScheduler();
    hdUnschedule(hMyCallback);
	for(size_t i=0;i<data.devices.size();i++)
		hdDisableDevice(data.devices[i].handle);

    return 0;
}

/*****************************************************************************/

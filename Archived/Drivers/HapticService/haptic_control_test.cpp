#include "JSONService.h"
using namespace std;

class HapticControlTestService : public JSONService
{
public:
	HapticControlTestService() {}
	virtual const char* Name() const { return "HapticControlTest"; }
	virtual const char* Description() const { return "Tests force feedback"; }
	virtual bool OnMessage(AnyCollection& message) { 
		if(message["type"] != string("MultiHapticState")) {
			printf("Invalid type member\n");
			return false;
		}
		if(message["devices"].size()==1) {
			cout<<"Force: "<<message["devices"][0]["force"]<<endl;
			//only one device
			AnyCollection cmd;
			cmd["type"] = string("HapticForceCommand");
			cmd["device"] = 0;
			cmd["enabled"] = 1;
			cmd["center"].resize(3);
			cmd["center"][0] = 0;
			cmd["center"][1] = 0;
			cmd["center"][2] = 0;
			cmd["linear"] = -50.0;
			SendMessage(cmd);
		}
		else {
			cout<<"Force: "<<message["devices"][0]["force"]<<endl;
			//two devices
			AnyCollection cmd;
			cmd["type"] = string("HapticForceCommand");
			cmd["device"] = 0;
			cmd["enabled"] = 1;
			cmd["overrideContinuousForceLimits"] = 1;
			cmd["center"].resize(3);
			cmd["center"][0] = message["devices"][1]["position"][0];
			cmd["center"][1] = message["devices"][1]["position"][1];
			cmd["center"][2] = message["devices"][1]["position"][2];
			cmd["velocityCenter"].resize(3);
			cmd["velocityCenter"][0] = message["devices"][1]["velocity"][0];
			cmd["velocityCenter"][1] = message["devices"][1]["velocity"][1];
			cmd["velocityCenter"][2] = message["devices"][1]["velocity"][2];
			cmd["linear"] = -50.0;
			cmd["damping"] = -3.0;
			SendMessage(cmd);
		}
		return true;
	}
};

int main(int argc,const char* argv[])
{
	HapticControlTestService service;
	int i = service.ParseCommandLineOptions(argc,argv);
	if(i < 0) return 1;
	if(i != argc) {
		printf("Warning, extra command line arguments...\n");
		service.PrintCommandLineOptions();
	}
	if(!service.Connected()) {
		if(!service.OpenClient()) {
			fprintf(stderr,"Error opening client socket\n");
			return 1;
		}
	}
	service.RunUntilKeypress();
	return 0;
}

/*****************************************************************************/

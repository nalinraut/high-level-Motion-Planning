#include <sspp/Service.h>
#include <KrisLibrary/Timer.h>
using namespace std;

class EchoService : public SSPP::Service
{
public:
	Timer timer;
	double lastTime;
	EchoService() : lastTime(0) {}
	virtual const char* Name() const { return "Echo"; }
	virtual const char* Description() const { return "Prints messages to console"; }
	virtual bool OnMessage(AnyCollection& message) { 
		cout<<message<<endl;
		double t = timer.ElapsedTime();
		printf("Time between messages: %g\n",t-lastTime);
		lastTime = t;
		return true;
	}
};

int main(int argc,const char* argv[])
{
	EchoService service;
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
	SSPP::RunForever(service);
	return 0;
}

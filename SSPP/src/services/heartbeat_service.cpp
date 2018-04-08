#include <sspp/Service.h>
#include <sstream>
using namespace std;

class HeartbeatService : public SSPP::Service
{
public:
  AnyCollection msg;
  HeartbeatService(double rate)
  {
    sleepTime = 1.0/rate;
    msg = 1;
  }
  int Process() {
    SendMessage(msg);
    return SSPP::Service::Process();
  }

  virtual int ParseCommandLineOption(int argc,const char** argv,int index) {
    if(0==strcmp(argv[index],"--r") || 0==strcmp(argv[index],"--rate")) {
      double rate = atof(argv[index+1]);
      sleepTime = 1.0/rate;
      return index+2;
    }
    else if(0==strcmp(argv[index],"-m") || 0==strcmp(argv[index],"--message")) {
      stringstream ss(argv[index+1]);
      ss >> msg;
      if(!ss) return -1;
      return index+2;
    }
    else return SSPP::Service::ParseCommandLineOption(argc,argv,index);
  }
  virtual void PrintCommandLineOptions() const {
    SSPP::Service::PrintCommandLineOptions();
    printf(" -r,--rate rate: sets the rate (in Hz), default 1.\n");
    printf(" -m,--message fn: sets the heartbeat message, default 1\n");
  }
};




int main(int argc,const char* argv[])
{
  HeartbeatService service(1);
  int i = service.ParseCommandLineOptions(argc,argv);
  if(i < 0) return 1;
  if(i !=  argc) {
    printf("Warning, extra command line arguments...\n");
    service.PrintCommandLineOptions();
  }
  
  if(!service.Connected()) {
    if(!service.OpenServer()) {
      fprintf(stderr,"Error opening server socket\n");
      return 1;
    }
  }
  SSPP::RunForever(service);
  return 0;
}

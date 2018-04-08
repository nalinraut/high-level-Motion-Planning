#include <sspp/Send.h>
#include <sspp/Recv.h>
#include <sstream>

namespace SSPP {

using namespace std;

int SendService::Process()
{
	int res = Service::Process();
	if(res < 0) return res;
	if(index < (int)messages.size()) {
		if(!SendMessage(messages[index])) {
			printf("Error sending message %d\n",index);
			return -1;
		}
		index++;
		if(loop && index == (int)messages.size())
			index = 0;
	}
	return res;
}

int SendService::ParseCommandLineOption(int argc,const char** argv,int index)
{
	if(0==strcmp(argv[index],"-m") || 0==strcmp(argv[index],"--message")) {
		stringstream ss(argv[index+1]);
		AnyCollection msg;
		ss >> msg;
		if(!ss) {
			printf("Could not parse message %s\n",argv[index+1]);
			return -1;
		}
		messages.push_back(msg);
		return index+2;
	}
	else if( 0==strcmp(argv[index],"--loop")) {
		loop = true;
		return index+1;
	}
	else return Service::ParseCommandLineOption(argc,argv,index);
}

void SendService::PrintCommandLineOptions() const
{
	Service::PrintCommandLineOptions();
	printf(" -m, --message msg: adds a message to the message queue\n");
	printf(" -loop: repeats the message queue infinitely\n");
}

bool SendService::AddMessage(const string& str)
{
	stringstream ss(str);
	AnyCollection msg;
	ss >> msg;
	if(!ss) {
		printf("Could not parse message %s\n",str.c_str());
		return false;
	}
	messages.push_back(msg);
	return true;
}

void SendService::AddMessage(const AnyCollection& msg)
{
	messages.push_back(msg);
}


bool Send(const char* addr,const AnyCollection& msg)
{
	SendService service;
	service.AddMessage(msg);
	if(!service.OpenClient(addr)) return false;
	if(!service.OnStart()) return false;
	while(service.index < 1) {
	  if(service.Process() < 0) {
	    service.OnStop();
	    return false;
	  }
	}
	service.OnStop();
	return true;
}

bool Send(const char* addr,const std::vector<AnyCollection>& msgs)
{
	SendService service;
	service.messages = msgs;
	if(!service.OpenClient(addr)) return false;
	if(!service.OnStart()) return false;
	while(service.index < (int)msgs.size()) {
	  if(service.Process() < 0) {
	    service.OnStop();
	    return false;
	  }
	}
	service.OnStop();
	return true;

}

bool Send(File& f,const AnyCollection& msg)
{
  stringstream ss;
  ss<<msg;
  int length = (int)ss.str().length();
  assert(sizeof(int)==4);
  if(!f.WriteData(&length,4)) 
    return false;
  return f.WriteData(ss.str().c_str(),length);
  //return f.WriteString(ss.str().c_str());
}

bool Send(File& f,const std::vector<AnyCollection>& msgs)
{
  for(size_t i=0;i<msgs.size();i++) {
    stringstream ss;
    ss<<msgs[i];
    int length = (int)ss.str().length();
    assert(sizeof(int)==4);
    if(!f.WriteData(&length,4)) 
      return false;
    if(!f.WriteData(ss.str().c_str(),length)) 
      return false;
    //if(!f.WriteString(ss.str().c_str())) return false;
  }
  return true;
}


bool Recv(const char* addr,AnyCollection& msg,double timeout)
{
  vector<AnyCollection> messages(1);
  if(!Recv(addr,messages,timeout)) return false;
  msg = messages[0];
  return true;
}

bool Recv(const char* addr,std::vector<AnyCollection>& msgs,double timeout) 
{
  RecvService service((int)msgs.size());
  if(timeout == 0) {
    RunForever(service);
  }
  else  {
    service.OnStart();
    Timer timer;
    while(timer.ElapsedTime() < timeout) {
      int n = service.Process();
      if(service.messages.size() == msgs.size()) break;
      if(n < 0) {
	//read error
	service.OnStop();
	return false;
      }
    }
    service.OnStop();
  }
  if(service.messages.size() != msgs.size()) return false;
  std::swap(msgs,service.messages);
  return true;
}



} //namespace SSPP

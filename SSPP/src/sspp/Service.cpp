#include <sspp/Service.h>
#include <sspp/Config.h>
#include <signal.h>
using namespace std;

#ifdef  _WIN64 
#pragma warning (disable:4996)
#endif

#ifdef _WIN32
#pragma warning (disable:4996)
#endif

#include <cstdio>
#include <cassert>
#include <iostream>
#include <KrisLibrary/utils/SignalHandler.h>
#ifdef _WIN32
#include <conio.h>
#else
#include <stdio.h>
#include <sys/select.h>
#include <termios.h>
int _kbhit() {
    static const int STDIN = 0;
    static bool initialized = false;

    if (! initialized) {
        // Use termios to turn off line buffering
        termios term;
        tcgetattr(STDIN, &term);
        term.c_lflag &= ~ICANON;
        tcsetattr(STDIN, TCSANOW, &term);
        setbuf(stdin, NULL);
        initialized = true;
    }

    timeval timeout;
    fd_set rdset;

    FD_ZERO(&rdset);
    FD_SET(STDIN, &rdset);
    timeout.tv_sec  = 0;
    timeout.tv_usec = 0;

    return select(STDIN + 1, &rdset, NULL, NULL, &timeout);  
}
#endif
#include <KrisLibrary/Timer.h>


namespace SSPP {

Service::Service()
	:tolerateReadErrors(false),onlyProcessNewest(false),useDeltaProtocol(false),sleepTime(0),waitTime(SSPP_CONNECTION_WAIT_TIME)
{}

Service::~Service()
{
	Disconnect();
}

bool Service::OpenServer(const char* host,double timeout,int maxClients)
{
	if(timeout <= 0) timeout = Math::Inf;
	printf("%s: Connecting as server to %s...\n",Name(),host);
	worker = new SocketPipeWorker(host,true,timeout);
	//TODO: do we need to modify these values for slow readers?  Usually hitting the end of the queue means something has gone wrong
	worker->reader.queueMax = SSPP_READ_QUEUE_SIZE;
	worker->writer.queueMax = SSPP_WRITE_QUEUE_SIZE;
	if(!worker->Start()) return false;
	printf("...Connection successful\n");
	return true;
}

bool Service::OpenClient(const char* host,double timeout)
{
	if(timeout <= 0) timeout = Math::Inf;
	printf("%s: Connecting as client to %s...\n",Name(),host);
	worker = new SocketPipeWorker(host,false,timeout);
	//TODO: do we need to modify these values for slow readers?  Usually hitting the end of the queue means something has gone wrong
	worker->reader.queueMax = SSPP_READ_QUEUE_SIZE;
	worker->writer.queueMax = SSPP_WRITE_QUEUE_SIZE;
	if(!worker->Start()) return false;
	printf("...Connection successful\n");
	return true;
}

bool Service::Connected() const { return worker != NULL; }

bool Service::Disconnect()
{
	if(!worker) { return false; }
	worker->Stop();
	worker = NULL;
	return true;
}

int Service::NumClients() const
{
	if(!worker) return -1;
	const SocketServerTransport* server = dynamic_cast<const SocketServerTransport*>(&*worker->transport);
	if(!server) return -1;
	return (int)server->clientsockets.size();
}

bool Service::WaitForMessage(AnyCollection& message,double timeout)
{
  if(!worker) {
    fprintf(stderr,"%s::WaitForMessage(): Not connected\n",Name());
    return false;
  }
 
  Timer timer;
  while(timer.ElapsedTime() < timeout) {
    if(!worker->initialized) {
      fprintf(stderr,"%s::WaitForMessage(): Abnormal disconnection\n",Name());
      return false;
    }
    //read new messages
    if(worker->UnreadCount() > 0) {
      if(onlyProcessNewest) {
	string str = worker->Newest();
	stringstream ss(str);
	AnyCollection msg;
	if(!msg.read(ss)) {
	  fprintf(stderr,"%s::WaitForMessage(): Got an improperly formatted string\n",Name());
	  cout<<"String = \""<<str<<"\""<<endl;
	  return false;
	}
	if(!OnMessage(msg)) {
	  fprintf(stderr,"%s::WaitForMessage(): OnMessage returned false\n",Name());
	  return false;
	}
	message = msg;
	return true;
      }
      else {
	vector<string> msgs = worker->New();
	for(size_t i=0;i<msgs.size();i++) {
	  stringstream ss(msgs[i]);
	  AnyCollection msg;
	  if(!msg.read(ss)) {
	    fprintf(stderr,"%s::WaitForMessage(): Got an improperly formatted string\n",Name());
	    return false;
	  }
	  if(!OnMessage(msg)) {
	    fprintf(stderr,"%s::WaitForMessage(): OnMessage returned false\n",Name());
	    return false;
	  }
	}
	message = msgs[0];
	return true;
      }
      ThreadSleep(SSPP_MESSAGE_WAIT_TIME);
    }
  }
  return false;
}

int Service::Process()
{
	if(!worker) {
		fprintf(stderr,"%s::Process(): Not connected\n",Name());
		return -1;
	}
	if(!worker->initialized) {
	  fprintf(stderr,"%s::Process(): Abnormal disconnection\n",Name());
	  return -1;
	}
	//read new messages
	if(worker->UnreadCount() > 0) {
		if(onlyProcessNewest) {
		  string str = worker->Newest();
			stringstream ss(str);
			AnyCollection msg;
			if(!msg.read(ss)) {
				fprintf(stderr,"%s::Process(): Got an improperly formatted string\n",Name());
				cerr<<"  Offending string: \""<<str<<"\""<<endl;
				return -1;
			}
			if(!OnMessage(msg)) {
				fprintf(stderr,"%s::Process(): OnMessage returned false\n",Name());
				return -1;
			}
			return 1;
		}
		else {
			vector<string> msgs = worker->New();
			for(size_t i=0;i<msgs.size();i++) {
				stringstream ss(msgs[i]);
				AnyCollection msg;
				if(!msg.read(ss)) {
					fprintf(stderr,"%s::Process(): Got an improperly formatted string\n",Name());
					cerr<<"  Offending string: \""<<msgs[i]<<"\""<<endl;
					return -1;
				}
				if(!OnMessage(msg)) {
					fprintf(stderr,"%s::Process(): OnMessage returned false\n",Name());
					return -1;
				}
			}
			return (int)msgs.size();
		}
	}
	return 0;
}

bool Service::SendMessage(AnyCollection& message)
{
	if(!worker) return false;
	if(!worker->WriteReady()) return false;
	stringstream ss;
	ss<<message;
	worker->Send(ss.str());
	return true;
}


int Service::ParseCommandLineOption(int argc,const char** argv,int i)
{
	if(0==strcmp(argv[i],"-h") || 0==strcmp(argv[i],"--help") ) {
		printf("%s: %s\n",Name(),Description());
		PrintCommandLineOptions();
		return i+1;
	}
	return -1;
}


int Service::ParseCommandLineOptions(int argc,const char** argv)
{
	bool open = false;
	bool server = true;
	string host = "tcp://localhost:4567";
	double timeout = 0;
	int i;
	for(i=1;i<argc;i++) {
		if(argv[i][0]=='-') {
			if(0==strcmp(argv[i],"-h") || 0==strcmp(argv[i],"--help") ) {
				printf("%s: %s\n",Name(),Description());
				PrintCommandLineOptions();
				return i+1;
			}
			else if(0==strcmp(argv[i],"-s") ||0==strcmp(argv[i],"--server")) {
				open = true;
				server = true;
				host = argv[i+1];
				i++;
			}
			else if(0==strcmp(argv[i],"-c") ||0==strcmp(argv[i],"--client")) {
				open = true;
				server = false;
				host = argv[i+1];
				i++;
			}
			else if(0==strcmp(argv[i],"-p") ||0==strcmp(argv[i],"--port")) {
				open = true;
				server = true;
				host = "tcp://localhost:";
				host += string(argv[i+1]);
				i++;
			}
			else if(0==strcmp(argv[i],"-t") ||0==strcmp(argv[i],"--timeout")) {
				timeout = atof(argv[i+1]);
				i++;
			}
			else {
				int newindex = ParseCommandLineOption(argc,argv,i);
				if(newindex < 0) {
					printf("%s: %s\n",Name(),Description());
					PrintCommandLineOptions();
					return -1;
				}
				i = newindex - 1;
			}
		}
		else break;
	}
	if(host.find("tcp://") != 0)
		host = "tcp://"+host;
	if(open) {
		if(server) {
			if(!OpenServer(host.c_str(),timeout)) return -1;
		}
		else {
			if(!OpenClient(host.c_str(),timeout)) return -1;
		}
	}
	return i;
}

static const char* OPTIONS_STRING = "OPTIONS:\n\
 -h,--help: prints this help message.\n\
 -s,--server address: hosts a server on the given address (default localhost:4567).\n\
 -c,--client address: connects to a server on the given address.\n\
 -p,--port port: hosts a server at localhost:port\n\
 -t,--timeout time: sets the timeout\n";


void Service::PrintCommandLineOptions() const
{
	printf(OPTIONS_STRING);
}

bool g_doKillService = false;
bool killService() { return g_doKillService; }
struct MySignalHandler : public SignalHandler
{
  virtual void OnRaise(int signum) { g_doKillService = true;  }
};

bool trueCondition() { return !g_doKillService; }

bool kbhitCondition() { return _kbhit() != 0 || g_doKillService; }


bool RunUntil(Service& s,bool (*condition)())
{
  if(!s.OnStart()) return false;
  while(!condition()) {
    int n=s.Process();
    if(g_doKillService) break;
    if(n < 0) {
      if(!s.tolerateReadErrors) {
	s.OnStop();
	return false;
      }
      else ThreadSleep(s.waitTime);
    }
    if(n == 0) {
      if(s.sleepTime<=0) ThreadYield();
      else ThreadSleep(s.sleepTime);
    }
  }
  s.OnStop();
  return true;
}

bool RunWhile(Service& s,bool (*condition)())
{
  if(!s.OnStart()) return false;
  while(condition()) {
    int n=s.Process();
    if(g_doKillService) break;
    if(n < 0) {
      if(!s.tolerateReadErrors) {
	s.OnStop();
	return false;
      }
      else ThreadSleep(s.waitTime);
    }
    if(n == 0) {
      if(s.sleepTime<=0) ThreadYield();
      else ThreadSleep(s.sleepTime);
    }
  }
  s.OnStop();
  return true;
}



bool RunForever(Service& s)
{
  MySignalHandler handler;
  handler.SetCurrent(SIGINT);
  return RunWhile(s,trueCondition);
}


bool RunUntilKeypress(Service& s)
{
  MySignalHandler handler;
  handler.SetCurrent(SIGINT);
  return RunUntil(s,kbhitCondition);
}


bool RunForever(vector<Service*>& s)
{
  MySignalHandler handler;
  handler.SetCurrent(SIGINT);
  return RunWhile(s,trueCondition);
}

bool RunUntil(vector<Service*>& s,bool (*condition)())
{
  if(s.empty()) return false;
  for(size_t i=0;i<s.size();i++) {
    if(!s[i]->OnStart()) {
      for(size_t j=0;j<=i;j++)
	s[j]->OnStop();
      return false;
    }
  }
  Timer timer;
  vector<double> nextProcessTimes(s.size(),0.0);
  while(!condition()) {
    double t = timer.ElapsedTime();
    bool fired = false;
    for(size_t i=0;i<s.size();i++) {
      if(nextProcessTimes[i] <= t) {
	fired = true;
	int n=s[i]->Process();
	if(n < 0) {
	  if(!s[i]->tolerateReadErrors) {
	    for(size_t j=0;j<s.size();j++)
	      s[j]->OnStop();
	    return false;
	  }
	  else nextProcessTimes[i] = t+s[i]->waitTime;
	}
	if(n == 0) {
	  if(s[i]->sleepTime<=0) nextProcessTimes[i] = t;
	  else nextProcessTimes[i] = t+s[i]->sleepTime;
	}
      }
    }
    if(fired) {
      double minNextTime = nextProcessTimes[0];
      for(size_t i=1;i<s.size();i++)
	minNextTime = Min(minNextTime,nextProcessTimes[1]);

      if(minNextTime == t)
	ThreadYield();
      else if(minNextTime > t) 
	ThreadSleep(minNextTime - t);
    }
  }
  for(size_t j=0;j<s.size();j++)
    s[j]->OnStop();
  return true;
}

bool RunWhile(vector<Service*>& s,bool (*condition)())
{
  if(s.empty()) return false;
  for(size_t i=0;i<s.size();i++) {
    if(!s[i]->OnStart()) {
      for(size_t j=0;j<=i;j++)
	s[j]->OnStop();
      return false;
    }
  }
  Timer timer;
  vector<double> nextProcessTimes(s.size(),0.0);
  while(condition()) {
    double t = timer.ElapsedTime();
    bool fired = false;
    for(size_t i=0;i<s.size();i++) {
      if(nextProcessTimes[i] <= t) {
	fired = true;
	int n=s[i]->Process();
	if(n < 0) {
	  if(!s[i]->tolerateReadErrors) {
	    for(size_t j=0;j<s.size();j++)
	      s[j]->OnStop();
	    return false;
	  }
	  else nextProcessTimes[i] = t+s[i]->waitTime;
	}
	if(n == 0) {
	  if(s[i]->sleepTime<=0) nextProcessTimes[i] = t;
	  else nextProcessTimes[i] = t+s[i]->sleepTime;
	}
      }
    }
    if(fired) {
      double minNextTime = nextProcessTimes[0];
      for(size_t i=1;i<s.size();i++)
	minNextTime = Min(minNextTime,nextProcessTimes[1]);

      if(minNextTime == t)
	ThreadYield();
      else if(minNextTime > t) 
	ThreadSleep(minNextTime - t);
    }
  }
  for(size_t j=0;j<s.size();j++)
    s[j]->OnStop();
  return true;
}

bool RunUntilKeypress(vector<Service*>& s)
{
  return RunUntil(s,kbhitCondition);
}


} //namespace SSPP

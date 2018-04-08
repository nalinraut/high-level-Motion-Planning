#include "JSONService.h"
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
#include <conio.h>
#include <Timer.h>


JSONService::JSONService()
	:tolerateReadErrors(false),onlyProcessNewest(false),useDeltaProtocol(false),sleepTime(0),waitTime(0.1)
{}

JSONService::~JSONService()
{
	Disconnect();
}

bool JSONService::OpenServer(const char* host,double timeout,int maxClients)
{
	if(timeout <= 0) timeout = Math::Inf;
	printf("%s: Connecting as server to %s...\n",Name(),host);
	worker = new SocketPipeWorker(host,true,timeout);
	if(!worker->Start()) return false;
	printf("...Connection successful\n");
	return true;
}

bool JSONService::OpenClient(const char* host,double timeout)
{
	if(timeout <= 0) timeout = Math::Inf;
	printf("%s: Connecting as client to %s...\n",Name(),host);
	worker = new SocketPipeWorker(host,false,timeout);
	if(!worker->Start()) return false;
	printf("...Connection successful\n");
	return true;
}

bool JSONService::Connected() const { return worker != NULL; }

bool JSONService::Disconnect()
{
	if(!worker) { return false; }
	worker->Stop();
	worker = NULL;
	return true;
}

int JSONService::Process()
{
	if(!worker) {
		fprintf(stderr,"%s::Process(): Not connected\n",Name());
		return -1;
	}
	//read new messages
	if(worker->NewMessageCount() > 0) {
		Timer timer;
		if(onlyProcessNewest) {
			stringstream ss(worker->NewestMessage());
			AnyCollection msg;
			ss>>msg;
			if(!ss) {
				fprintf(stderr,"%s::Process(): Got an improperly formatted string\n",Name());
				return -1;
			}
			if(!OnMessage(msg)) {
				fprintf(stderr,"%s::Process(): OnMessage returned false\n",Name());
				return -1;
			}
			return 1;
		}
		else {
			vector<string> msgs = worker->NewMessages();
			for(size_t i=0;i<msgs.size();i++) {
				stringstream ss(msgs[i]);
				AnyCollection msg;
				ss>>msg;
				if(!ss) {
					fprintf(stderr,"%s::Process(): Got an improperly formatted string\n",Name());
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

bool JSONService::RunForever()
{
	if(!Start()) return false;
	while(true) {
		int n=Process();
		if(n < 0) {
			if(!tolerateReadErrors) return false;
			else ThreadSleep(waitTime);
		}
		if(n == 0) {
			if(sleepTime<=0) ThreadYield();
			else ThreadSleep(sleepTime);
		}
	}
	return true;
}

bool JSONService::RunUntil(bool (*condition)())
{
	if(!Start()) return false;
	while(!condition()) {
		int n=Process();
		if(n < 0) {
			if(!tolerateReadErrors) return false;
			else ThreadSleep(waitTime);
		}
		if(n == 0) {
			if(sleepTime<=0) ThreadYield();
			else ThreadSleep(sleepTime);
		}
	}
	return true;
}

bool JSONService::RunWhile(bool (*condition)())
{
	if(!Start()) return false;
	while(condition()) {
		int n=Process();
		if(n < 0) {
			if(!tolerateReadErrors) return false;
			else ThreadSleep(waitTime);
		}
		if(n == 0) {
			if(sleepTime<=0) ThreadYield();
			else ThreadSleep(sleepTime);
		}
	}
	return true;
}

bool JSONService::RunUntilKeypress()
{
	if(!Start()) return false;
	while (!_kbhit()) {
		int n=Process();
		if(n < 0) {
			if(!tolerateReadErrors) return false;
			else ThreadSleep(waitTime);
		}
		if(n == 0) {
			if(sleepTime<=0) ThreadYield();
			else ThreadSleep(sleepTime);
		}
	}
	return true;
}

bool JSONService::SendMessage(AnyCollection& message)
{
	if(!worker) return false;
	if(!worker->WriteReady()) return false;
	stringstream ss;
	ss<<message;
	worker->SendMessage(ss.str());
	return true;
}


int JSONService::ParseCommandLineOption(int argc,const char** argv,int i)
{
	if(0==strcmp(argv[i],"-h") || 0==strcmp(argv[i],"--help") ) {
		printf("%s: %s\n",Name(),Description());
		PrintCommandLineOptions();
		return i+1;
	}
	return -1;
}


int JSONService::ParseCommandLineOptions(int argc,const char** argv)
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

const char* OPTIONS_STRING = "OPTIONS:\n\
 -h,--help: prints this help message.\n\
 -s,--server address: hosts a server on the given address (default localhost:4567).\n\
 -c,--client address: connects to a server on the given address.\n\
 -p,--port port: hosts a server at localhost:port\n\
 -t,--timeout time: sets the timeout\n";


void JSONService::PrintCommandLineOptions() const
{
	printf(OPTIONS_STRING);
}
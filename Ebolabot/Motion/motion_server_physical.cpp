#include "motion_physical.cpp"
#include <signal.h>

void run_motion_server(const char*);

int main(int argc,char** argv)
{
  const char* serverAddr = "tcp://192.168.0.101:8001";
  if(argc >= 2)
    serverAddr = argv[1];
  signal(SIGPIPE,SIG_IGN);
  run_motion_server(serverAddr);
}

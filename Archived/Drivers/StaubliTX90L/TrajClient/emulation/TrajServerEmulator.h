#ifndef TRAJ_SERVER_EMULATOR_H
#define TRAJ_SERVER_EMULATOR_H

#include "CS8Emulator.h"
#include <Timer.h>

class TrajServerEmulator
{
 public:
  TrajServerEmulator(CS8Emulator& emulator);
  ~TrajServerEmulator();
  bool Init(int recvport);
  void Cleanup();
  void Process();

 private:
  bool AcceptConnection();
  bool CloseConnections();
  bool CloseConnection(int index);
  bool PollRecv(int index,std::vector<std::string>& messages);
  bool ParseMessage(int index,const std::string& str);
  std::string DispatchCommand(const std::string& cmd,const std::string& args);

  CS8Emulator& emulator;
  Timer virtualTimer;
  int sockserv;
  std::vector<int> sockrecv;
  int nReceived,nSent,nCalls;
  std::string partialInput;
};


#endif

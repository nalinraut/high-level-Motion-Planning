#ifndef SSPP_RECV_H
#define SSPP_RECV_H

#include "Service.h"

namespace SSPP {

/** @brief A simple service that receives up to max messages and stores
 * them in a vector.
 */
class RecvService : public Service
{
public:
  std::vector<AnyCollection> messages;
  int max;
  
  RecvService(int _max) : max(_max) {}
  virtual ~RecvService() {}
  virtual const char* Name() const { return "Recv"; }
  virtual const char* Description() const { return "Receives a fixed sequence of one or more messages"; }
  virtual int Process() { int res = Service::Process(); if((int)messages.size()>=max) return -1; return res; }
  virtual bool OnMessage(AnyCollection& msg) { messages.push_back(msg); return true; }
};

bool Recv(const char* addr,AnyCollection& msg,double timeout=0);
bool Recv(const char* addr,std::vector<AnyCollection>& msgs,double timeout=0);

 bool Recv(File& f,const AnyCollection& msg,double timeout=0);
 bool Recv(File& f,const std::vector<AnyCollection>& msgs,double timeout=0);

} //namespace SSPP

#endif

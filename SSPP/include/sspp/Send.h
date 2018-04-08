#ifndef SSPP_SEND_H
#define SSPP_SEND_H

#include "Service.h"

namespace SSPP {

class SendService : public Service
{
public:
	std::vector<AnyCollection> messages;
	int index;
	bool loop;

	SendService():index(0),loop(false) {}
	virtual ~SendService() {}
	virtual const char* Name() const { return "Send"; }
	virtual const char* Description() const { return "Sends a fixed sequence of one or more messages"; }
	virtual int Process();
	virtual int ParseCommandLineOption(int argc,const char** argv,int index);
	virtual void PrintCommandLineOptions() const;
	bool AddMessage(const std::string& str);
	void AddMessage(const AnyCollection& msg);
};

bool Send(const char* addr,const AnyCollection& msg);
bool Send(const char* addr,const std::vector<AnyCollection>& msgs);

bool Send(File& f,const AnyCollection& msg);
bool Send(File& f,const std::vector<AnyCollection>& msgs);

} //namespace SSPP

#endif

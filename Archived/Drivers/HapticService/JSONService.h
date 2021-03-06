#ifndef JSON_SERVICE_H
#define JSON_SERVICE_H

#include <Timer.h>
#include <utils/AsyncIO.h>
#include <utils/AnyCollection.h>

/** @brief Base class for a JSON service.
 *
 * Default operation:
 *
 * MyJSONService service;
 * //optionally, set options here
 * bool res=service.OpenServer(host); //if server mode
 * //bool res=service.OpenClient(host); //if client mode
 * if(!res) { return error; }
 * res = service.RunForever(); 
 * if(!res) { return error; }
 *
 * Subclasses must override OnMessage(). Optionally, they may override
 * Name() and Description() to print nicer error messages.
 */
class JSONService
{
public:
	JSONService();
	~JSONService();
	///Reads the command line options and returns the index of the first argument that cannot be processed.
	///Returns -1 on error.
	///Can configure with -s host, -c host, -p port, -t timeout, and -h prints the options message.
	virtual int ParseCommandLineOptions(int argc,const char** argv);
	///Subclass can add options here.  Returns next index
	virtual int ParseCommandLineOption(int argc,const char** argv,int index);
	///Prints the set of command line options that works with this service
	virtual void PrintCommandLineOptions() const;
	///Opens this service on a server socket.  If timeout is given, then it returns false if the
	///socket could not be opened.  If maxClients is given, it specifies the maximum number of
	///clients connected at any one time (default 1)
	bool OpenServer(const char* host="tcp://localhost:4567",double timeout=0,int maxClients=1);
	///Opens this service on a client socket.  If timeout is given, then it returns false if the
	///socket could not be opened.
	bool OpenClient(const char* host="tcp://localhost:4567",double timeout=0);
	///Returns true if this socket is connected
	bool Connected() const;
	///Returns the number of connected clients, if this is a server socket
	int NumClients() const;
	///Disconnects the socket
	bool Disconnect();
	///Running the service: if you are manually calling Process(), call this before doing so
	virtual bool Start() { return true; }
	///Running the service: can manually call Process in a polling loop. Returns -1 on
	///error, or otherwise returns the number of processed messages
	virtual int Process();
	///Running the service: runs forever or until a force quit
	bool RunForever();
	///Running the service: runs until the condition returns true
	bool RunUntil(bool (*condition)());
	///Running the service: runs while the condition returns true
	bool RunWhile(bool (*condition)());
	///Running the service: runs until a key on the keyboard is pressed
	bool RunUntilKeypress();

	///Subclasses: optionally override this
	virtual const char* Name() const { return "Service"; }
	///Subclasses: optionally override this
	virtual const char* Description() const { return "Provides some JSON service"; }
	///Subclasses: override this
	virtual bool OnMessage(AnyCollection& message) { return true; }
	///Subclasses: call this to send a message
	bool SendMessage(AnyCollection& message);

	SmartPointer<SocketPipeWorker> worker;
	///Configuration option: ignore read errors (useful for servers where the client may suddenly disconnect)
	bool tolerateReadErrors;
	///Configuration option: if multiple reads are pending, only do the newest (default false)
	bool onlyProcessNewest;
	///Configuration option: read from a DynamicStructureDelta stream
	bool useDeltaProtocol;
	///Configuration option: in RunX() functions, how long to sleep between Process() calls (default 0)
	double sleepTime;
	///Configuration option: in RunX() functions, how long to sleep on a socket that errors out (default 0.1)
	double waitTime;
};

#endif
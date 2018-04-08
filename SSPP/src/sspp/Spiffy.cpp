#include <sspp/Spiffy.h>
#include <sspp/Topic.h>
using namespace std;

namespace SSPP {

string server = SSPP_DEFAULT_TOPIC_SERVER;
string context = ".";

Mutex globalMutex;
map<string,vector<AnySpiffy*> > pushVariables;
map<string,int> writeOnly;

class SpiffyPushService:public Service
{
public:
  SpiffyPushService(const char* host)
  {
    if(!OpenClient(host)) return;
    for(map<string,vector<AnySpiffy*> >::iterator i=pushVariables.begin();i!=pushVariables.end();i++) {
      AnyCollection msg;
      msg["type"] = string("subscribe");
      msg["path"] = i->first;
      SendMessage(msg);
    }
  }
  virtual ~SpiffyPushService() {
    for(map<string,vector<AnySpiffy*> >::iterator i=pushVariables.begin();i!=pushVariables.end();i++) {
      AnyCollection msg;
      msg["type"] = string("unsubscribe");
      msg["path"] = i->first;
      SendMessage(msg);
    }      
  }
  virtual const char* Name() const { return "SpiffyPushService"; }
  virtual const char* Description() const { return ""; }
  virtual bool OnMessage(AnyCollection& message) { 
    ScopedLock lock(globalMutex);
    //push message
    if(pushVariables.size()==1) {
      for(size_t i=0;i<pushVariables.begin()->second.size();i++)
	pushVariables.begin()->second[i]->_updated(message);
    }
    else {
      string path;
      message["path"].as(path);
      map<string,vector<AnySpiffy*> >::iterator it = pushVariables.find(path);
      if(it == pushVariables.end()) {
	cout<<"Invalid subscription message path \""<<path<<"\""<<endl;
	return false;
      }
      for(size_t i=0;i<it->second.size();i++)
	it->second[i]->_updated(message["data"]);
    }
    return true;
  }
  void AddPush(const string& path,AnySpiffy* var) {
    if(pushVariables.count(path) == 0) {
      pushVariables[path] = vector<AnySpiffy*>(1,var);
      //subscribe
      AnyCollection msg;
      msg["type"] = string("subscribe");
      msg["path"] = path;
      SendMessage(msg);
    }
    else
      pushVariables[path].push_back(var);
  }
  void RemovePush(const string& path,AnySpiffy* var)
  {
    Assert(pushVariables.count(path) != 0);
    vector<AnySpiffy*>& vars = pushVariables[path];
    for(size_t i=0;i<vars.size();i++) {
      if(vars[i] == var) {
	vars.erase(vars.begin()+i);
	i--;
      }
    }
    if(vars.empty()) {
      //unsubscribe
      AnyCollection msg;
      msg["type"] = string("unsubscribe");
      msg["path"] = path;
      SendMessage(msg);
      pushVariables.erase(pushVariables.find(path));
    }
  }
};

class SpiffySynchronizedService : public Service
{
public:
  File client;
  int myID;
  SpiffySynchronizedService(const char* host)
  {
    if(!client.Open(host,FILECLIENT)) return;
    //note: possible conflict if multiple services start up simultaneously
    AnyCollection result;
    if(!Get("._spiffy.clientCounter",result)) {
      myID = 0;
    }
    else {
      int nc;
      if(!result.as(nc)) {
	FatalError("Address %s._spiffy.clientCounter not an int\n",host);
      }
      myID = nc;
    }
    Set("._spiffy.clientCounter",AnyCollection(myID+1));
    for(map<string,int>::iterator i=writeOnly.begin();i!=writeOnly.end();i++)
      MarkExclusiveWrite(i->first,true,i->second);
  }
  virtual ~SpiffySynchronizedService() {
    for(map<string,int>::iterator i=writeOnly.begin();i!=writeOnly.end();i++)
      MarkExclusiveWrite(i->first,false,i->second);
  }
  bool MarkExclusiveWrite(const string& path,bool enabled,int id)
  {
    if(enabled) {
      if(!IsWriteEnabled(path,id)) {
	printf("MarkExclusiveWrite(%s,%d,%d): Write permissions for this ID not set\n",path.c_str(),(int)enabled,id);
	return false;
      }
      AnyCollection msg;
      msg["client"] = myID;
      msg["instance"] = id;
      Set(string("._spiffy.writeOnly")+path,msg);
      writeOnly[path] = id;
      return true;
    }
    else {
      if(writeOnly.count(path) == 0) return true;
      if(writeOnly[path] != id) {
	printf("MarkExclusiveWrite(%s,%d,%d): Write permissions for this ID not set\n",path.c_str(),(int)enabled,id);
	return false;
      }
      AnyCollection msg;
      msg["type"] = string("delete");
      msg["path"] = string(".spiffy.writeOnly")+path;
      SendMessage(msg);
      writeOnly.erase(writeOnly.find(path));
      return true;
    }
  }
  bool IsWriteEnabled(const string& path,int id) {
    if(writeOnly.count(path) != 0) {
      if(writeOnly[path] != id) return false;
      else return true;
    }
    AnyCollection msg;
    if(!Get(string("._spiffy.writeOnly")+path,msg)) return true;
    int client,instance;
    if(!msg["client"].as(client) || msg["instance"].as(instance)) return true;
    if(client != myID || id != instance) return false;
    return true;
  }
  bool Set(const string& path,const AnyCollection& value) {
    AnyCollection msg;
    msg["type"] = string("set");
    msg["path"] = path;
    msg["data"] = value;
    SendMessage(msg);    
    return true;
  }
  bool Get(const string& path,AnyCollection& result) {
    AnyCollection msg;
    msg["type"] = string("get");
    msg["path"] = path;
    SendMessage(msg);    
    return WaitForMessage(result);
  }
};

void SetSpiffyServer(const char* addr)
{
  server = addr;
}

const char* GetSpiffyServer()
{
  return server.c_str();
}

bool spiffyStarted = false;
SmartPointer<SpiffySynchronizedService> syncService;
SmartPointer<SpiffyPushService> pushService;
int globalID = 0;

Thread pushThread;
bool pushThreadKill = false;

//destructor
class SpiffyGlobalContext
{
public:
  SpiffyGlobalContext() {}
  ~SpiffyGlobalContext() { if(spiffyStarted) StopSpiffy(); }
};

SpiffyGlobalContext gContext;

bool pushKilled()
{
  return pushThreadKill;
}

void* pushThreadFunc(void*)
{
  pushThreadKill = false;
  RunUntil(*pushService,pushKilled);
  return NULL;
}


bool StartSpiffy(const char* context)
{
  SetSpiffyContext(context);
  pushService = new SpiffyPushService(server.c_str());
  syncService = new SpiffySynchronizedService(server.c_str());
  if(!pushService->Connected() || !syncService->Connected()) {
    printf("Spiffy server on address %s not started\n",server.c_str());
    pushService = NULL;
    syncService = NULL;
    return false;
  }
  spiffyStarted = true;
  pushThread = ThreadStart(pushThreadFunc,NULL);
  return true;
}

void StopSpiffy()
{
  pushThreadKill = true;
  ThreadJoin(pushThread);
  pushService->Disconnect();
  pushService = NULL;
  syncService = NULL;
  spiffyStarted = false;
}

void SetSpiffyContext(const char* ctx)
{
  ScopedLock lock(globalMutex);
  if(ctx == NULL) context="";
  else context=ctx;
}




AnySpiffy::AnySpiffy(const char* _name)
  :name(_name),path(string(context)+"."+string(name)),pushMode(false),newValue(false)
{
  ScopedLock lock(globalMutex);
  id = globalID++;
}

AnySpiffy::~AnySpiffy()
{
  if(writeOnly.count(path) && writeOnly[path]==id) {
    syncService->MarkExclusiveWrite(path,false,id);
  }
  SetPullMode();
}

bool AnySpiffy::SetExclusiveWrite(bool enabled)
{
  ScopedLock lock(globalMutex);
  return syncService->MarkExclusiveWrite(path,enabled,id);
}

bool AnySpiffy::CanWrite()
{
  ScopedLock lock(globalMutex);
  return syncService->IsWriteEnabled(path,id);
}

bool AnySpiffy::CanRead()
{
  return true;
}

void AnySpiffy::SetPullMode()
{
  if(!pushMode) return;
  pushMode = false;
  pushService->RemovePush(path,this);
}
 
void AnySpiffy::SetPushMode()
{
  if(pushMode) return;
  pushMode = true;
  pushService->AddPush(path,this);
  newValue = false;
}

bool AnySpiffy::HasNewValue()
{
  return newValue;
}




bool AnySpiffy::set(const AnyCollection& _value)
{
  ScopedLock lock(globalMutex);
  if(!syncService->IsWriteEnabled(path,id)) return false;
  value = _value;
  return syncService->Set(path,_value);
}

bool AnySpiffy::get(AnyCollection& _value)
{
  if(pushMode) {
    _value = value;
    newValue = false;
    return true;
  }
  else {
    return syncService->Get(path,_value);
  }
}

void AnySpiffy::_updated(const AnyCollection& _value)
{
  value = _value;
  newValue = true;
}


} //namespace SSPP

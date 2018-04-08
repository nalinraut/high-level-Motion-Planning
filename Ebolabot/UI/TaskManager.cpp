#include "TaskManager.h"
#include <sspp/Topic.h>

static int gNumTaskManagers = 0;

TaskManager::TaskManager(const char* system_state_addr)
: current(NULL),
  systemStateClient(system_state_addr)
{ 
  if(gNumTaskManagers != 0) {
    FatalError("Can only have 1 active TaskManager");
  }
  gNumTaskManagers ++;
  printf("TaskManager: publishing tasks to .controller.task on %s\n",system_state_addr);
  systemStateClient.Listen(".robot.t");
  systemStateClient.OnStart();
}

TaskManager::~TaskManager() {
  if(current) current->Stop();
  gNumTaskManagers --;
  systemStateClient.OnStop();
}

string TaskManager::Status() { if(current==NULL) return ""; return current->Status(); }

void TaskManager::AvailableModes(vector<string>& modes)
{
  std::map<std::string,TaskGenerator*>& reg = TaskGenerator::Registry();
  modes.resize(0);
  for(std::map<std::string,TaskGenerator*>::iterator i=reg.begin();i!=reg.end();i++)
    modes.push_back(i->first);
}

bool TaskManager::SetMode(const string& str,bool reinitialize)
{
  if(current && current->Name()==str && !reinitialize) return true; //no change
  if(current) {
    GLGUIPlugin* plugin = current->GLPlugin();
    if(plugin) plugin->Stop();
    current->Stop();
    current = NULL;

    //stop current task by sending empty message
    AnyCollection msg;
    msg["type"] = string("");
    msg["task_request_time"] = systemStateClient.Get(".robot.t");
    systemStateClient.Set(".controller.task",msg);
  }
  if(TaskGenerator::Registry().count(str)==0) {
    current = NULL;
    printf("Got an unknown mode %s\n",str.c_str());
    return false;
  }
  current = TaskGenerator::Registry()[str];
  if(initialized.count(str)==0) {
    printf("Initializing mode %s\n",str.c_str());
    if(current->Init(&commonWorld)) 
      initialized[str] = true;
    else
      initialized[str] = false;
  }
  else if(reinitialize) {
    if(!initialized[str]) {
      printf("Re-initializing mode %s\n",str.c_str());
      if(current->Init(&commonWorld)) 
        initialized[str] = true;
      else
        initialized[str] = false;
    }
  }
  else {
    //dont re-initialize failed modes
  }
  if(!initialized[str]) return false;
  current->Start();
  GLGUIPlugin* plugin = current->GLPlugin();
  if(plugin) {
    plugin->Start();
    plugin->sleepIdleTime = 0;
    plugin->wantsRefresh = true;
  }
  return true;
}

GLGUIPlugin* TaskManager::GLPlugin()
{
  if(!current) return NULL; return current->GLPlugin();
}

void TaskManager::Update()
{
  systemStateClient.Process();
  if(!current) return;
  AnyCollection msg;
  current->GetTask(msg);
  if(msg.null()) return;
  msg["task_request_time"] = systemStateClient.Get(".robot.t");
  //printf("Setting message...\n");
  systemStateClient.Set(".controller.task",msg);
  //printf("Running Process...\n");
  systemStateClient.Process();
  //printf("Done.\n");
}

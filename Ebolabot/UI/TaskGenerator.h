#ifndef TASK_GENERATOR_H
#define TASK_GENERATOR_H

#include "GLGUIPlugin.h"
#include <KrisLibrary/utils/AnyCollection.h>
#include <Modeling/World.h>

class TaskGenerator
{
 public:
  TaskGenerator():world(NULL) {}
  virtual ~TaskGenerator() {}
  ///Override this to return the appropriate name
  virtual std::string Name() { return "TaskGenerator"; }

  ///Called once at beginning of program, or if you want to do a hard reinitialization
  virtual bool Init(RobotWorld* _world);

  ///Called on start
  virtual bool Start()=0;

  ///Called repeatedly to retrieve status of task generator
  virtual std::string Status()=0;

  ///Called repeatedly to retrieve current task to be served
  ///Note: the current commanded configuration is placed in the world
  virtual void GetTask(AnyCollection& c)=0;

  ///Called repeatedly to retrieve current task to be served in JSON format
  ///default just prints out the result from GetTask(AnyCollection)
  virtual std::string GetTask();

  ///Called on stop
  virtual void Stop() {}
  
  ///Optional: return a plugin to be used on the GL viewport. 
  ///value is a reference, not a new object.
  virtual GLGUIPlugin* GLPlugin() { return NULL; }

  static void RegisterMe(TaskGenerator* me);
  static std::map<std::string,TaskGenerator*>& Registry();

  RobotWorld* world;
};

#endif

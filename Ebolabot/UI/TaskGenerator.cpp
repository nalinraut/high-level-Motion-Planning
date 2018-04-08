#include "TaskGenerator.h"


std::map<std::string,TaskGenerator*>& TaskGenerator::Registry()
{
  static std::map<std::string,TaskGenerator*> registeredGenerators;
  return registeredGenerators;
}

void TaskGenerator::RegisterMe(TaskGenerator* me)
{
  Registry()[me->Name()] = me;
}

bool TaskGenerator::Init(RobotWorld* _world)
{
  world = _world;
  return true;
}


std::string TaskGenerator::GetTask()
{
  AnyCollection c;
  GetTask(c);
  stringstream ss;
  c.write(ss);
  return ss.str();
}


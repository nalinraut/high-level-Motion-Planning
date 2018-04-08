#include "TaskController.h"


std::map<std::string,TaskController*>& TaskController::Registry()
{
  static std::map<std::string,TaskController*> registeredControllers;
  return registeredControllers;
}

void TaskController::RegisterMe(TaskController* me)
{
  Registry()[me->TaskName()] = me;
}

void TaskController::SetWorld(RobotWorld* _world)
{
  world = _world;
  robot = world->robots[0];
}

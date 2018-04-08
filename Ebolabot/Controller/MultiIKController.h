#ifndef MULTI_IK_CONTROLLER_H
#define MULTI_IK_CONTROLLER_H

#include "TaskController.h"
#include <KrisLibrary/robotics/IK.h>

class MultiIKController : public TaskController
{
 public:
  bool started;
  vector<IKGoal> goals;

  MultiIKController();
  virtual ~MultiIKController() {}
  std::string TaskName() { return "MultiIK"; }
  bool Start(AnyCollection& params);
  std::string Status();
  void Stop();
  virtual void DrawGL();
};

#endif

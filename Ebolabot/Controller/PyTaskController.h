#ifndef PY_TASK_CONTROLLER_H
#define PY_TASK_CONTROLLER_H

#include "TaskController.h"

#if HAVE_PYTHON
#include <Python.h>
#else
typedef void PyObject;
#endif //HAVE_PYTHON

class PyTaskController : public TaskController
{
 public:
  PyTaskController(const std::string& module);
  virtual ~PyTaskController();
  virtual std::string TaskName();
  virtual bool Start(AnyCollection& params);
  virtual std::string Status();
  virtual void Stop();
  virtual void DrawGL();

  std::string moduleName;
  PyObject *object;
};

///Registers all Python task controllers in the given path
bool RegisterPyTaskControllers(const char* path="");
void UnregisterPyTaskControllers();

#endif

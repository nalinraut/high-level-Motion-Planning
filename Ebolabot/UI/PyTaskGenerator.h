#ifndef PY_TASK_GENERATOR_H
#define PY_TASK_GENERATOR_H

#include "TaskGenerator.h"

#if HAVE_PYTHON
#include <Python.h>
#else
typedef void PyObject;
typedef void PyThreadState;
#endif //HAVE_PYTHON

class PyTaskGenerator : public TaskGenerator
{
 public:
  PyTaskGenerator(const std::string& module);
  virtual ~PyTaskGenerator();
  bool LoadPyObject();
  virtual std::string Name();
  virtual bool Init(RobotWorld* world);
  virtual bool Start();
  virtual std::string Status();
  virtual std::string GetTask();
  virtual void GetTask(AnyCollection& c);
  virtual void Stop();
  virtual GLGUIPlugin* GLPlugin();

  std::string moduleName;
  PyObject *object;
  PyThreadState *threadState;
  bool hasGUI;
  GLGUIPlugin* gui;
  bool hasError;
  PyObject* jsonModule;
};

///Registers all Python task generators in the given path
bool RegisterPyTaskGenerators(const char* path="");
///Destroys all Python task generators
void ClearPyTaskGenerators();

#endif

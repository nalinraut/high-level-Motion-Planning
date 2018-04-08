#include "PyTaskController.h"
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/utils/stringutils.h>
using namespace std;

#if HAVE_PYTHON

PyTaskController::PyTaskController(const std::string& module)
{
  moduleName = module;
  object = NULL;

  printf("PyTaskController(%s): Initializing...\n",module.c_str());
  PyObject* pModule = PyImport_ImportModule(module.c_str());
  if(!pModule) {
    printf("Error opening Python module %s, check paths or import errors\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    moduleName = "";
    return;
  }
  PyObject* makeFunc = PyObject_GetAttrString(pModule,"make");
  if(!makeFunc) {
    printf("Python module %s does not have make() function\n",moduleName.c_str());
    Py_DECREF(pModule);
    moduleName = "";
    return;
  }
  if (!PyCallable_Check(makeFunc)) {
    printf("Python module %s has a make object that is not a function\n",moduleName.c_str());
    Py_DECREF(pModule);
    Py_DECREF(makeFunc);
    moduleName = "";
    return;
  }
  PyObject* arglist = Py_BuildValue("()");
  object = PyEval_CallObject(makeFunc, arglist);
  Py_DECREF(arglist);
  Py_DECREF(makeFunc);
  if(!object) {
    //exception thrown
    PyErr_Print();
    Py_DECREF(pModule);
    moduleName = "";
    return;
  }
  Py_DECREF(pModule);
}

PyTaskController::~PyTaskController()
{
  if(object != NULL) {
    PyObject* retVal = PyObject_CallMethod(object,"close",NULL);
    Py_DECREF(retVal);
  }
  Py_XDECREF(object);
}

std::string PyTaskController::TaskName()
{
  if(object == NULL) return "";
  PyObject* retVal = PyObject_CallMethod(object,"taskName",NULL);
  if(retVal==NULL) return "";
  else {
    char* c=PyString_AsString(retVal);
    if(c == NULL) {
      printf("PyTaskController(%s): Warning, taskName() does not return a string\n",moduleName.c_str());
      Py_DECREF(retVal);
      return "";
    }
    std::string s(c);
    Py_DECREF(retVal);
    return s;
  }
}

PyObject* ValueToObject(const AnyValue& value)
{
  if(value.empty()) {
    Py_RETURN_NONE;
  }
  const std::type_info* type = &value.type();
  if(type == &typeid(bool)) {
    if(*AnyCast<bool>(&value)) Py_RETURN_TRUE;
    else Py_RETURN_FALSE;
  }
  if(type == &typeid(char)) {
    return PyString_FromStringAndSize(&(*AnyCast<char>(&value)),1);
  }
  else if(type == &typeid(int)) 
    return PyInt_FromLong(*AnyCast<int>(&value));
  else if(type == &typeid(unsigned int)) 
    return PyInt_FromLong(*AnyCast<unsigned int>(&value));
  else if(type == &typeid(float)) 
    return PyFloat_FromDouble((double)*AnyCast<float>(&value));
  else if(type == &typeid(double)) 
    return PyFloat_FromDouble(*AnyCast<double>(&value));
  else if(type == &typeid(std::string)) 
    return PyString_FromString(AnyCast<string>(&value)->c_str());
  else
    FatalError("Conversion of objects of type %s not supported",type->name());
  return NULL;
}

PyObject* CollectionToObject(AnyCollection& collection)
{
  if(collection.null()) {
    Py_INCREF(Py_None);
    return Py_None;
  }
  else if(collection.isvalue()) {
    return ValueToObject((const AnyValue&)(collection));
  }
  else if(collection.isarray()) {
    PyObject* retval = PyList_New(collection.size());
    for(size_t i=0;i<collection.size();i++)
      PyList_SetItem(retval,i,CollectionToObject(collection[i]));
    return retval;
  }
  else {
    PyObject* retval = PyDict_New();
    std::vector<SmartPointer<AnyCollection> > values;
    std::vector<AnyKeyable> keys;
    collection.enumerate_keys(keys);
    collection.enumerate(values);
    Assert(keys.size()==values.size());
    for(size_t i=0;i<keys.size();i++)
      PyDict_SetItem(retval,ValueToObject(keys[i].value),CollectionToObject(*values[i]));
    return retval;
  }
}

bool PyTaskController::Start(AnyCollection& params)
{
  PyObject* paramDict = CollectionToObject(params);
  PyObject* retVal = PyObject_CallMethod(object,"start","(N)",paramDict);
  Py_DECREF(paramDict);
  if(retVal==NULL) {
    fprintf(stderr,"PyTaskController(%s): calling start() returned a NULL pointer\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    return false;
  }
  if(retVal == Py_True) {
    Py_DECREF(retVal);
    return true;
  }
  else {
    Py_DECREF(retVal);
    return false;
  }
}

std::string PyTaskController::Status()
{
  if(object == NULL) return "";
  PyObject* retVal = PyObject_CallMethod(object,"status",NULL);
  if(retVal==NULL) {
    if(PyErr_Occurred()) PyErr_Print();
    return "";
  }
  else {
    char* c=PyString_AsString(retVal);
    if(c == NULL) {
      printf("PyTaskController(%s): Warning, status() does not return a string\n",moduleName.c_str());
        if(PyErr_Occurred()) PyErr_Print();
      Py_DECREF(retVal);
      return "";
    }
    std::string s(c);
    Py_DECREF(retVal);
    return s;
  }
}

void PyTaskController::Stop()
{
  PyObject* retVal = PyObject_CallMethod(object,"stop",NULL);
  Py_XDECREF(retVal);
  return;
}

void PyTaskController::DrawGL()
{
  PyObject* retVal = PyObject_CallMethod(object,"drawGL",NULL);
  Py_XDECREF(retVal);
  return;
}

bool EndsWith( const std::string &fullString, const std::string &ending)
{
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

static vector<SmartPointer<PyTaskController> > g_pyTaskControllerList;

///Registers all Python task controllers in the given path
bool RegisterPyTaskControllers(const char* path)
{
  vector<string> items;
  FileUtils::ListDirectory(path,items);
  vector<string> modules;

  cout<<"RegisterPyTaskControllers: Reading Python files in path "<<path<<endl;
  for(size_t i=0;i<items.size();i++) {
    cout<<"  List item: "<<i<<": "<<items[i]<<endl;
    if(EndsWith(items[i],".py") && items[i] != "task_controller.py") {
      StripExtension(items[i]);
      modules.push_back(items[i]);
    }
  }

  if(!modules.empty()) {
    //create modules and registerthem
    for(size_t i=0;i<modules.size();i++) {
      SmartPointer<PyTaskController> c = new PyTaskController(modules[i]);
      if(c->moduleName.empty()) {
	printf("RegisterPyTaskControllers: Failed to register Python module %s\n",c->moduleName.c_str());
	printf("Press enter to continue...\n");
	getchar();
	continue;
      }
      g_pyTaskControllerList.push_back(c);
      TaskController::RegisterMe(c);
    }
    printf("RegisterPyTaskControllers: Registered the following tasks:\n");
    for(size_t i=0;i<g_pyTaskControllerList.size();i++) {
      printf("  %s: %s.py\n",g_pyTaskControllerList[i]->TaskName().c_str(),g_pyTaskControllerList[i]->moduleName.c_str());
    }
  }
}

void UnregisterPyTaskControllers()
{
  g_pyTaskControllerList.resize(0);
}

#else // HAVE_PYTHON

PyTaskController::PyTaskController(const std::string& module)
{
}

PyTaskController::~PyTaskController()
{
}

std::string PyTaskController::TaskName()
{
  return "";
}

bool PyTaskController::Start(AnyCollection& params)
{
  return false;
}

std::string PyTaskController::Status()
{
  return "";
}

void PyTaskController::Stop()
{
}

bool RegisterPyTaskControllers(const char* path)
{
  printf("Python API is not available, cannot register Python controllers\n");
  return false;
}

void UnregisterPyTaskControllers()
{
}

#endif // HAVE_PYTHON

#include "Common/SWIGPyObject.h"

#include "PyTaskGenerator.h"
#include <KrisLibrary/utils/fileutils.h>
#include <KrisLibrary/utils/stringutils.h>
using namespace std;

#if HAVE_PYTHON

//helper class for acquiring python interpreter thread
class PyThreadGetter
{
public:
  PyThreadState*& state;
  PyThreadGetter(PyThreadState*& _state):state(_state)
  {
    if(state) PyEval_RestoreThread(state);
  }
  ~PyThreadGetter() {
    if(state) state = PyEval_SaveThread();
  }
};

GLGUIPlugin* gPyTaskGenObject = NULL;

static PyObject *
pytaskgen_viewport(PyObject *self, PyObject *args)
{
  if(gPyTaskGenObject==NULL) return NULL;
  AnyCollection res;
  const Camera::Viewport& vp = gPyTaskGenObject->viewport;
  res["perspective"] = vp.perspective;
  res["scale"] = vp.scale;
  res["x"] = vp.x;
  res["y"] = vp.y;
  res["w"] = vp.w;
  res["h"] = vp.h;
  res["n"] = vp.n;
  res["f"] = vp.f;
  res["xform"].resize(16);
  Matrix4 cmat = vp.xform;
  const Real* matdata = cmat;
  for(int i=0;i<16;i++) {
    res["xform"][i] = matdata[i];
  } 

  stringstream ss;
  ss<<res;
  return PyString_FromString(ss.str().c_str());
}

static PyObject *
pytaskgen_refresh(PyObject *self, PyObject *args)
{
  if(gPyTaskGenObject==NULL) return NULL;
  gPyTaskGenObject->Refresh();
  Py_RETURN_NONE;
}

static PyObject *
pytaskgen_idlesleep(PyObject *self, PyObject *args)
{
  if(gPyTaskGenObject==NULL) return NULL;
  double s;
  if(!PyArg_ParseTuple(args,"f",&s)) return NULL;
  gPyTaskGenObject->SleepIdle(s);
  Py_RETURN_NONE;
}

static PyObject *
pytaskgen_click_ray(PyObject *self, PyObject *args)
{
  if(gPyTaskGenObject==NULL) return NULL;
  double mx,my;
  if(!PyArg_ParseTuple(args,"ff",&mx,&my)) return NULL;
  Vector3 s,d;
  gPyTaskGenObject->ClickRay(mx,my,s,d);
  PyObject* pys = Py_BuildValue("(fff)",s.x,s.y,s.z);
  PyObject* pyd = Py_BuildValue("(fff)",d.x,d.y,d.z);  
  return PyTuple_Pack(2,pys,pyd);
}


static PyMethodDef PytaskgenMethods[] = {
    {"viewportJson",  pytaskgen_viewport, METH_VARARGS,"Return viewport string"},
    {"idlesleep",  pytaskgen_idlesleep, METH_VARARGS,"Sleeps the idle function"},
    {"refresh",  pytaskgen_refresh, METH_VARARGS,"Requests a refresh"},
    {"click_ray",  pytaskgen_click_ray, METH_VARARGS,"Returns a ray in world space"},
    {NULL, NULL, 0, NULL}        /* Sentinel */
};

const char* pyWindowString = "from klampt import Viewport \n\
import pytaskgen \n\
class PyGLViewHack: \n\
    def __init__(self): \n\
        self.h,self.w = 480,640 \n\
    def toViewport(self): \n\
        res = Viewport() \n\
        str = pytaskgen.viewportJson() \n\
        res.fromJson(str) \n\
        self.h = res.h \n\
        self.w = res.w \n\
        return res\n\
class PyWindow: \n\
    def __init__(self): \n\
        self.view = PyGLViewHack() \n\
    def idlesleep(self,seconds): \n\
        pytaskgen.idlesleep(seconds) \n\
    def refresh(self): \n\
        pytaskgen.refresh() \n\
    def click_ray(self,mx,my): \n\
        return pytaskgen.click_ray(mx,my) \n\
";

bool pyTaskGenInitialized = false;
PyObject* pyTaskGenWindow = NULL;

PyObject* LoadPyTaskGen(GLGUIPlugin* plugin)
{
  Assert(!pyTaskGenInitialized);
  Assert(gPyTaskGenObject == NULL);
  gPyTaskGenObject = plugin;
  Py_InitModule("pytaskgen", PytaskgenMethods);
  if(PyRun_SimpleString(pyWindowString) < 0) {
    printf("Error running PyWindow string\n");
    if(PyErr_Occurred()) PyErr_Print();
    return NULL;
  }
  /* get sys.modules dict */
  PyObject* sys_mod_dict = PyImport_GetModuleDict();
  /* get the __main__ module object */
  PyObject* main_mod = PyMapping_GetItemString(sys_mod_dict, "__main__");
  /* call the class inside the __main__ module */
  pyTaskGenWindow = PyObject_CallMethod(main_mod, "PyWindow", "");

  if(pyTaskGenWindow  == NULL) {
    printf("Error creating PyWindow object\n");
    if(PyErr_Occurred()) PyErr_Print();
    return NULL;
  }
  pyTaskGenInitialized = true;
  return pyTaskGenWindow;
}

void UnloadPyTaskGen()
{
  Py_DECREF(pyTaskGenWindow);
  pyTaskGenWindow = NULL;
  pyTaskGenInitialized = false;
  gPyTaskGenObject = NULL;
}

class PyGLGUIPlugin : public GLGUIPlugin
{
public:
  PyThreadState* threadState;
  string name;
  PyObject* plugin;
  PyObject* window;
  int old_mx,old_my;
  bool hasError;

  PyGLGUIPlugin(PyThreadState*_threadState,const string& _name,PyObject* _plugin) 
  : threadState(_threadState),name(_name),plugin(_plugin),window(NULL),old_mx(-1),old_my(-1),hasError(false) {  }
  ~PyGLGUIPlugin() { Stop(); }
  virtual void Start() {
    PyThreadGetter threadGetter(threadState);
    printf("GUI Plugin %s start\n",name.c_str());
    Assert(window == NULL);
    Assert(plugin != NULL);
    //need to set the plugin's window attribute to have a class with the
    //following callbacks 
    //- idlesleep(seconds)
    //- refresh()
    //- click_ray(mx,my)
    //- .view needs to be a class with the following callback
    //   - toViewport()

    window = LoadPyTaskGen(this);
    if(!window) {
      fprintf(stderr,"Strange, error creating the PyWindow object\n");
      hasError = true;
      abort();
      return;
    }
    int set = PyObject_SetAttrString(plugin,"window",window);
    if(set < 0) {
      fprintf(stderr,"Strange, could not set the plugin's window attribute\n");
      hasError = true;
      abort();
      return;
    }
    PyObject* view = PyObject_GetAttrString(window,"view");
    set = PyObject_SetAttrString(plugin,"view",view);
    if(set < 0) {
      hasError=true;
      abort();
    }

    //now initialize it
    PyObject* retVal = PyObject_CallMethod(plugin,"initialize",NULL);
    if(!retVal) {
      fprintf(stderr,"GUI Plugin %s Note: Python GLPluginBase's initialize function threw an exception\n",name.c_str());
      if(PyErr_Occurred()) PyErr_Print();
      getchar();
      hasError = true;
      return;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    if(!res) {
      fprintf(stderr,"GUI Plugin %s Note: Python GLPluginBase's initialize function returned False\n",name.c_str());
      getchar();
      return;
    }
  }
  virtual void Stop() {
    PyThreadGetter threadGetter(threadState);
    if(window) UnloadPyTaskGen();
    window = NULL;
    printf("GUI Plugin %s stop\n",name.c_str());
  }
  virtual void RenderWorld() {
    if(hasError) return;
    PyThreadGetter threadGetter(threadState);

    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"display",NULL);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      abort();
      return;
    }
    Py_DECREF(retVal);
  }
  virtual void RenderScreen() {
    if(hasError) return;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"display_screen",NULL);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      abort();
      return;
    }
    Py_DECREF(retVal);
  }
  virtual bool OnIdle() { 
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"idle",NULL);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      abort();
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
  virtual bool OnViewportChange() { 
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"reshapefunc","(ii)",viewport.w,viewport.h);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      abort();
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
  virtual bool OnCameraChange() { return false; }
  virtual bool OnMouseClick(int button,int state,int mx,int my) { 
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    old_mx = mx;
    old_my = my;
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"mousefunc","(iiii)",button,state,mx,my);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      abort();
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
  virtual bool OnMouseMove(int mx,int my) {  
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    bool res=false;
    if(old_mx >= 0) {
      int dx = mx - old_mx;
      int dy = my - old_my;
      Assert(plugin != NULL);
      PyObject* retVal = PyObject_CallMethod(plugin,"motionfunc","(iiii)",mx,my,dx,dy);
      if(!retVal) {
	if(PyErr_Occurred()) PyErr_Print();
	hasError = true;
	return false;
      }
      res = (retVal != Py_False);
      Py_DECREF(retVal);
    }
    old_mx = mx;
    old_my = my;
    return res;
  }
  virtual bool OnKeyDown(const std::string& key) { 
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"keyboardfunc","(sii)",key.c_str(),old_mx,old_my);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
  virtual bool OnKeyUp(const std::string& key) { 
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"keyboardupfunc","(sii)",key.c_str(),old_mx,old_my);
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
  virtual bool OnButtonPress(const std::string& button) { return OnEvent("button",button); }
  virtual bool OnButtonToggle(const std::string& button,int checked) {
    stringstream ss; ss<<button<<" "<<checked;
    return OnEvent("toggle",ss.str());
  }
  virtual bool OnWidgetValue(const std::string& widget,const std::string& value) { 
    stringstream ss; ss<<widget<<" "<<value;
    return OnEvent("widget",ss.str());
  }
  virtual bool OnDevice(const std::string& name,const std::string& data) { 
    stringstream ss; ss<<name<<" "<<data;
    return OnEvent("device",ss.str());
  }
  bool OnEvent(const std::string& type,const std::string& args) {
    if(hasError) return false;

    PyThreadGetter threadGetter(threadState);
    Assert(plugin != NULL);
    PyObject* retVal = PyObject_CallMethod(plugin,"eventfunc","(ss)",type.c_str(),args.c_str());
    if(!retVal) {
      if(PyErr_Occurred()) PyErr_Print();
      hasError = true;
      return false;
    }
    bool res = (retVal != Py_False);
    Py_DECREF(retVal);
    return res;
  }
};


PyTaskGenerator::PyTaskGenerator(const std::string& module)
{
  moduleName = module;
  object = NULL;
  threadState = NULL;
  hasGUI = false;
  gui = NULL;
  hasError = false;
  jsonModule = NULL;

  PyObject* sjson = PyString_FromString("json");
  jsonModule = PyImport_Import(sjson);
  Py_XDECREF(sjson);
}

bool PyTaskGenerator::LoadPyObject()
{
  Assert(threadState == NULL);
  if(object != NULL) {
    PyObject* retVal = PyObject_CallMethod(object,"close",NULL);
    Py_DECREF(retVal);
  }
  Py_XDECREF(object);
  
  printf("PyTaskGenerator(%s): Loading python module...\n",moduleName.c_str());
  PyObject* pModule = PyImport_ImportModule(moduleName.c_str());
  if(!pModule) {
    printf("Error opening Python module %s, check paths or import errors\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return false;
  }
  PyObject* makeFunc = PyObject_GetAttrString(pModule,"make");
  if(!makeFunc) {
    printf("Python module %s does not have make() function\n",moduleName.c_str());
    Py_DECREF(pModule);
    hasError = true;
    return false;
  }
  if (!PyCallable_Check(makeFunc)) {
    printf("Python module %s has a make object that is not a function\n",moduleName.c_str());
    Py_DECREF(pModule);
    Py_DECREF(makeFunc);
    hasError = true;
    return false;
  }
  PyObject* arglist = Py_BuildValue("()");
  object = PyEval_CallObject(makeFunc, arglist);
  Py_DECREF(arglist);
  Py_DECREF(makeFunc);
  if(!object) {
    //exception thrown
    PyErr_Print();
    Py_DECREF(pModule);
    hasError = true;
    return false;
  }
  Py_DECREF(pModule);
}

PyTaskGenerator::~PyTaskGenerator()
{    
  if(object != NULL) {
    if(threadState) PyEval_RestoreThread(threadState);
    PyObject* retVal = PyObject_CallMethod(object,"close",NULL);
    Py_DECREF(retVal);
  }
  Py_XDECREF(object);
  if(gui) delete gui;
  Py_XDECREF(jsonModule);
}

std::string PyTaskGenerator::Name()
{
  if(object == NULL) {
    //fprintf(stderr,"PyTaskGenerator: Object is NULL\n");
    return "";
  }
  PyThreadGetter threadGetter(threadState);
  PyObject* retVal = PyObject_CallMethod(object,"name",NULL);
  if(retVal==NULL) {
    fprintf(stderr,"Object does not appear to have name method?\n");
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return "";
  }
  else {
    char* c=PyString_AsString(retVal);
    if(c == NULL) {
      printf("PyTaskGenerator(%s): Warning, taskName() does not return a string\n",moduleName.c_str());
      Py_DECREF(retVal);
      hasError = true;
      return "";
    }
    std::string s(c);
    Py_DECREF(retVal);
    return s;
  }
}

bool PyTaskGenerator::Init(RobotWorld* world)
{   
  if(hasError || !object) {
    if(object) {
      printf("Warning: reloading Python module %s, was this intended?\n",moduleName.c_str());
    }
    if(!LoadPyObject()) {
      hasError = true;
      return false;
    }
  }
  if(threadState != NULL) {
    printf("Warning: thread state is not NULL?\n");
    getchar();
  }
  hasError = false;
  //get a python world
  PyObject* sklampt = PyString_FromString("klampt");
  PyObject* klamptModule = PyImport_Import(sklampt);
  Py_XDECREF(sklampt);
  
  printf("Constructing Klamp't WorldModel from RobotWorld pointer...\n");
  printf("  (this is a delicate operation: needs Swig's version in Klamp't to match\n");
  printf("  exactly with that in Common/SwigPyObject.h AND for a hack in Klamp't's\n");
  printf("  robotsim_wrap.cxx to handle NULL ptr arguments FIRST in _wrap_new_WorldModel)\n");
  PyObject* swigPyObject = SwigPyObject_New(world,NULL,0);
  PyObject* pyWorld = PyObject_CallMethod(klamptModule,"WorldModel","(O)",swigPyObject);
  Py_XDECREF(swigPyObject);
  printf("Done.\n");
  if(!pyWorld) {
    fprintf(stderr,"PyTaskGenerator(%s): couldn't construct klampt.WorldModel\n",moduleName.c_str());
    Py_XDECREF(klamptModule);
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return false;
  }
  Py_XDECREF(klamptModule);
  PyObject* retVal = PyObject_CallMethod(object,"init","(O)",pyWorld);
  Py_XDECREF(pyWorld);
  if(retVal==NULL) {
    fprintf(stderr,"PyTaskGenerator(%s): calling init() failed\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return false;
  }
  if(retVal == Py_True) {
    Py_DECREF(retVal);
    return true;
  }
  else {
    fprintf(stderr,"PyTaskGenerator(%s): init() returned false\n",moduleName.c_str());
    Py_DECREF(retVal);
    hasError = true;
    return false;
  }
}

bool PyTaskGenerator::Start()
{
  Assert(threadState == NULL);
  PyObject* retVal = PyObject_CallMethod(object,"start",NULL);
  if(retVal==NULL) {
    fprintf(stderr,"PyTaskGenerator(%s): calling start() failed\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return false;
  }
  if(retVal == Py_True) {
    Py_DECREF(retVal);
    threadState = PyEval_SaveThread();
    return true;
  }
  else {
    hasError = true;
    Py_DECREF(retVal);
    return false;
  }
}

std::string PyTaskGenerator::Status()
{
  if(object == NULL) return "";
  if(hasError) return "error";
  Assert(threadState != NULL);
  PyThreadGetter threadGetter(threadState);
  PyObject* retVal = PyObject_CallMethod(object,"status",NULL);
  if(retVal==NULL) return "";
  else {
    char* c=PyString_AsString(retVal);
    if(c == NULL) {
      printf("PyTaskGenerator(%s): Warning, status() does not return a string\n",moduleName.c_str());
      Py_DECREF(retVal);
      return "";
    }
    std::string s(c);
    Py_DECREF(retVal);
    return s;
  }
}

void PyTaskGenerator::Stop()
{
  if(hasError) return;
  Assert(threadState != NULL);
  PyEval_RestoreThread(threadState);
  PyObject* retVal = PyObject_CallMethod(object,"stop",NULL);
  threadState = NULL;
  if(retVal == NULL) {
    fprintf(stderr,"PyTaskGenerator(%s): calling stop() failed\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return;
  }
  Py_XDECREF(retVal);
  return;
}

void PyTaskGenerator::GetTask(AnyCollection& c)
{
  string s = GetTask();
  if(s.empty()) {
    c = AnyCollection();
    return;
  }
  stringstream ss(s);
  c.read(ss);
}

std::string PyTaskGenerator::GetTask()
{
  if(hasError) return "";
  Assert(threadState != NULL);
  PyThreadGetter getter(threadState);
  PyObject* retVal = PyObject_CallMethod(object,"get",NULL);
  if(retVal == NULL) {
    fprintf(stderr,"PyTaskGenerator(%s): calling get() failed\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    hasError = true;
    return "";
  }
  if(retVal == Py_None) {
    Py_XDECREF(retVal);
    return "";
  }
  //convert to json string
  PyObject* objstr = PyObject_CallMethod(jsonModule,"dumps","(N)",retVal);
  Py_XDECREF(retVal);
  if(objstr == NULL) {
    fprintf(stderr,"Can't run json.dumps?\n");
    if(PyErr_Occurred()) PyErr_Print();
    return NULL;
  }
  const char* str;
  if(!PyArg_Parse(objstr,"s",&str)) {
    fprintf(stderr,"json.dumps didn't return a string?\n");
    Py_XDECREF(objstr);
    return "";
  }
  string sres = str;
  Py_XDECREF(retVal);
  Py_XDECREF(objstr);
  return sres;
}


GLGUIPlugin* PyTaskGenerator::GLPlugin()
{
  if(hasGUI) return gui;
  if(hasError) return NULL;
  hasGUI = true;
  Assert(threadState);
  PyThreadGetter threadGetter(threadState);
  PyObject* retVal = PyObject_CallMethod(object,"glPlugin",NULL);
  if(retVal == NULL) {
    fprintf(stderr,"PyTaskGenerator(%s): calling glPlugin() failed\n",moduleName.c_str());
    if(PyErr_Occurred()) PyErr_Print();
    return NULL;
  }
  if(retVal == Py_None) {
    Py_DECREF(retVal);
    gui = NULL;
    return NULL;
  }
  gui = new PyGLGUIPlugin(threadState,moduleName,retVal);
  return gui;
}

bool EndsWith( const std::string &fullString, const std::string &ending)
{
  if (fullString.length() >= ending.length()) {
    return (0 == fullString.compare (fullString.length() - ending.length(), ending.length(), ending));
  } else {
    return false;
  }
}

static vector<SmartPointer<PyTaskGenerator> > g_pyTaskGeneratorList;

///Registers all Python task controllers in the given path
bool RegisterPyTaskGenerators(const char* path)
{
  vector<string> items;
  FileUtils::ListDirectory(path,items);
  vector<string> modules;

  cout<<"RegisterPyTaskGenerators: Reading Python files in path "<<path<<endl;
  for(size_t i=0;i<items.size();i++) {
    cout<<"  List item: "<<i<<": "<<items[i]<<endl;
    if(EndsWith(items[i],".py") && items[i] != "task_generator.py") {
      StripExtension(items[i]);
      modules.push_back(items[i]);
    }
  }

  if(!modules.empty()) {
    //create modules and registerthem
    for(size_t i=0;i<modules.size();i++) {
      SmartPointer<PyTaskGenerator> c = new PyTaskGenerator(modules[i]);
      if(c->moduleName.empty()) {
	printf("RegisterPyTaskGenerators: Failed to register Python module %s\n",c->moduleName.c_str());
	printf("Press enter to continue...\n");
	getchar();
	continue;
      }
      c->LoadPyObject();
      if(c->Name().empty()) {
        printf("RegisterPyTaskGenerators: Python module %s had error, or did not provide name() method\n",c->moduleName.c_str());
        printf("Press enter to continue...\n");
        getchar();
        continue;
      }
      g_pyTaskGeneratorList.push_back(c);

      TaskGenerator::RegisterMe(c);
    }
    printf("RegisterPyTaskGenerators: Registered the following tasks:\n");
    for(size_t i=0;i<g_pyTaskGeneratorList.size();i++) {
      printf("  %s: %s.py\n",g_pyTaskGeneratorList[i]->Name().c_str(),g_pyTaskGeneratorList[i]->moduleName.c_str());
    }
  }
}

void ClearPyTaskGenerators()
{
  g_pyTaskGeneratorList.clear();
}

#else // HAVE_PYTHON

PyTaskGenerator::PyTaskGenerator(const std::string& module)
{
}

PyTaskGenerator::~PyTaskGenerator()
{
}

std::string PyTaskGenerator::Name()
{
  return "";
}

bool PyTaskGenerator::Init(RobotWorld& world)
{
  return false;
}

bool PyTaskGenerator::Start()
{
  return false;
}

std::string PyTaskGenerator::GetTask()
{
  return "";
}

std::string PyTaskGenerator::Status()
{
  return "";
}

void PyTaskGenerator::Stop()
{
}

GLGUIPlugin* PyTaskGenerator::GLPlugin()
{
  return NULL;
}

bool RegisterPyTaskGenerators(const char* path)
{
  printf("Python API is not available, cannot register Python generator\n");
  return false;
}

#endif // HAVE_PYTHON

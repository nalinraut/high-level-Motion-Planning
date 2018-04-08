#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include "motion.h"
#include <Modeling/World.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <string>
#include <map>

/**@brief An abstract base class that should be filled out by task
 * controllers.  If you want your controller to be registered you must 
 * instantiate a global object of your controller and call RegisterMe(this)
 * in your constructor.
 *
 * The dispatcher will call Start() any time this task is requested, including
 * when the task's parameters change.  It will also call Status() at
 * arbitrary times to request the controller's status.
 *
 * If a new task is requested, the dispatcher will call Stop().
 *
 * If the dispatcher is run in visualization mode, the DrawGL() function will
 * be called to show the controller's status.  This can be overloaded to
 * do visual debugging.
 */
class TaskController
{
 public:
  TaskController() {}
  virtual ~TaskController() {}
  ///Returns the name of the task that this controller is supposed to run
  virtual std::string TaskName() { return "Task"; }
  ///Called by dispatcher to start the controller
  virtual bool Start(AnyCollection& params)=0;
  ///Called by dispatcher to monitor the status of the controller. Return
  ///results can be:
  ///- Empty string: not running / Stop() successful 
  ///- "ok": task is running, and everything is ok
  ///- "done": task was completed successfully
  ///- "unavailable": the task is not currently available
  ///- "invalid parameters": Start() was called with invalid parameters
  ///- "error": generic error
  ///- "incomplete": the controller internally determined that had to
  ///   stop, and the task was not achieved
  ///- "stopping": the controller is currently stopping (either Stop() was
  ///   called or the task was determined to need to stop).
  ///- "fatal": fatal error, shut down the robot
  virtual std::string Status()=0;
  ///Called by dispatcher to tell the controller to stop
  virtual void Stop()=0;

  ///Called by visual dispatcher to allow visual debugging of the task;
  ///optional and not used in practice.
  virtual void DrawGL() {}

  ///Called by dispatcher; allows the Klamp't world model to be shared
  ///among task controllers
  void SetWorld(RobotWorld* world);

  static void RegisterMe(TaskController* me);
  static std::map<std::string,TaskController*>& Registry();

  RobotWorld* world;
  Robot* robot;
};

#endif //TASK_CONTROLLER_H

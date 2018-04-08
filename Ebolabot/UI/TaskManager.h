#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#include "TaskGenerator.h"
#include <sspp/Topic.h>

/** @brief Convenient interface for the set of possible task generators.
 * This needs to be embedded into your chosen GUI framework.
 *
 * Your job is to set up the task generators first by calling
 * RegisterPyTaskGenerators("UI/TaskGenerators"), 
 * and then during your event loop:
 * - Call SetMode() whenever a mode change is requested
 * - Call Update() repeatedly to send tasks to the server; this can be done
 *   in an idle loop or at a fixed rate.
 * - Call GLPlugin() to get the current GLGUIPlugin, and if not NULL, 
 *   be sure to shuttle all user input/render/idle events to the
 *   resulting pointer (see its documentation for more information). 
 *   Make sure to obey the requested signals "wantsRefresh"
 *   and "sleepIdleTime" within your GUI framework.
 *
 * You may also wish to monitor Status() in case there are any errors
 * reported.
 */
class TaskManager
{
public:
  TaskManager(const char* system_state_addr);
  ~TaskManager();

  string Status();
  void AvailableModes(vector<string>& modes);
  bool SetMode(const string& str,bool reinitialize=true);
  GLGUIPlugin* GLPlugin();
  void Update();

  TaskGenerator* current;
  SSPP::MultiTopicListener systemStateClient;
  RobotWorld commonWorld;
  string oldStatus;
  map<string,bool> initialized;
};

#endif

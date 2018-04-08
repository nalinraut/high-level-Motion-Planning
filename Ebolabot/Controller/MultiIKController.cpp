#include "MultiIKController.h"
#include <Planning/PlannerObjective.h>
#include <View/ViewIK.h>
#include <IO/JSON.h>

MultiIKController::MultiIKController()
{
  started=false; 
  TaskController::RegisterMe(this);
}


bool MultiIKController::Start(AnyCollection& params)
{
  if(!robot) {
    printf("MultiIKController: Klamp't model not set, can't start\n");
    return false; 
  }

  size_t n=params["components"].size();
  if(n == 0) {
    printf("Got an empty multi-ik objective, stopping planning\n");
    goals.resize(0);
    sendPlannerObjectiveStr(NULL);
  }
  else {
    goals.resize(n);
    for(size_t i=0;i<n;i++) {
      if(!Convert(params["components"][(int)i],goals[i])) {
	printf("Error converting IK constraint %d\n",(int)i);
	sendPlannerObjectiveStr(NULL);
	return true;
      }
    }
    printf("Setting objective with %d IK constraints\n",(int)n);
    
    CompositeObjective* obj = new CompositeObjective;
    for(size_t i=0;i<n;i++) {
      IKObjective* ikobj = new IKObjective(robot);
      ikobj->ikGoal = goals[i];
      obj->Add(ikobj);
    }
    stringstream ss;
    bool res = SavePlannerObjective(obj,ss);
    if(!res) printf("Warning, SavePlannerObjective(CompositeObjective) failed! Not sending to robot\n");
    else sendPlannerObjectiveStr(ss.str().c_str());
  }
}

std::string MultiIKController::Status()
{
  if(!started) return "";
  if(plannerObjectiveValue() < 1e-3) return "done";
  return "ok";
}

void MultiIKController::Stop()
{
  stopPlanner();
  goals.resize(0);
  started = false;
}

void MultiIKController::DrawGL()
{
  ViewIKGoal vg;
  for(size_t i=0;i<goals.size();i++)
    vg.Draw(goals[i],*robot);
}

MultiIKController multi_ik;

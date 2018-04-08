#ifndef ANY_SPACE_H
#define ANY_SPACE_H

#include "XmlReader.h"
#include <planning/Geometric2DCSpace.h>
#include <planning/MultiRobot2DCSpace.h>
#include <math/random.h>
#include <GLdraw/GL.h>
#include <GLdraw/drawExtra.h>
using namespace GLDraw;
using namespace std;

struct PlanningQuery
{
  bool ReadFromXml(TiXmlElement* e);
  bool IsPointToPoint() const;
  void GetEndpoints(Config& start,Config& goal) const;

  struct EndpointCondition
  {
    vector<Config> configs;
  };

  string name;
  EndpointCondition start,goal;
};

struct DrawableCSpace
{
  bool ReadFromXml(TiXmlElement* e);
  operator CSpace* ();
  CSpace* operator -> ();
  void DrawStatic();
  //0 = normal, 1 = start, 2 = end, 3+ = other
  void DrawConfiguration(const Config& q,int type=0);
  //draw a small icon indicating a roadmap configuration
  void DrawRoadmapNode(const Config& q,int type=0);
  //draw an edge indicating a roadmap path
  void DrawRoadmapEdge(const Config& a,const Config& b,int type=0);

  SmartPointer<Geometric2DCSpace> pointSpace;
  SmartPointer<TranslatingRobot2DCSpace> translatingSpace;
  SmartPointer<RigidRobot2DCSpace> rigidSpace;
  SmartPointer<MultiRobot2DCSpace> multiSpace;

  vector<PlanningQuery> queries;
};




//draw static obstacles in the space
template <class Space>
void DrawStatic(Space& s);

//draw a configuration q
//0 = normal, 1 = start, 2 = end, 3+ = other
template <class Space>
void DrawConfiguration(Space& s,const Config& q,int type=0);

//draw a small icon indicating a roadmap configuration
template <class Space>
void DrawRoadmapNode(Space& s,const Config& q,int type=0);

//draw a small icon indicating a roadmap edge
//0 = normal, 1 = path, 2 = disabled, 3+ = other
template <class Space>
void DrawRoadmapEdge(Space& s,const Config& a,const Config& b,int type=0);






bool PlanningQuery::ReadFromXml(TiXmlElement* e)
{
  if(e->QueryStringAttribute("name",&name)!=TIXML_SUCCESS) {
    name = "Untitled";
  }
  start.configs.resize(0);
  goal.configs.resize(0);
  TiXmlElement* s=e->FirstChildElement("start");
  if(s) {
    TiXmlElement* c=s->FirstChildElement("config");
    while(c) {
      start.configs.resize(start.configs.size()+1);
      if(c->QueryValueAttribute("data",&start.configs.back())!=TIXML_SUCCESS) {
	return false;
      }
      c = c->NextSiblingElement("config");
    }
  }
  else printf("Warning, planning query has no start condition\n");
  TiXmlElement* g=e->FirstChildElement("goal");
  if(g) {
    TiXmlElement* c=g->FirstChildElement("config");
    while(c) {
      goal.configs.resize(goal.configs.size()+1);
      if(c->QueryValueAttribute("data",&goal.configs.back())!=TIXML_SUCCESS) {
	return false;
      }
      c = c->NextSiblingElement("config");
    }
  }
  else printf("Warning, planning query has no goal condition\n");
  return true;
}

bool PlanningQuery::IsPointToPoint() const
{
  return start.configs.size()==1 && goal.configs.size()==1;
}

void PlanningQuery::GetEndpoints(Config& qs,Config& qg) const
{
  qs = start.configs[RandInt(start.configs.size())];
  qg = goal.configs[RandInt(goal.configs.size())];
}




template <> void DrawStatic(Geometric2DCSpace& s) { s.DrawGL(); }
template <> void DrawStatic(RigidRobot2DCSpace& s) { s.DrawWorkspaceGL(); }
template <> void DrawStatic(TranslatingRobot2DCSpace& s) { s.DrawWorkspaceGL(); }
template <> void DrawStatic(MultiRobot2DCSpace& s) { s.DrawWorkspaceGL(); }


template <> void DrawConfiguration(Geometric2DCSpace& s,const Config& q,int type)
{
  glBegin(GL_POINTS);
  glVertex2d(q(0),q(1));
  glEnd();
}

template <> void DrawConfiguration(RigidRobot2DCSpace& s,const Config& q,int type)
{
  s.DrawRobotGL(q);
}
template <> void DrawConfiguration(TranslatingRobot2DCSpace& s,const Config& q,int type)
{
  s.DrawRobotGL(q);
}
template <> void DrawConfiguration(MultiRobot2DCSpace& s,const Config& q,int type)
{
  for(size_t i=0;i<s.robots.size();i++)
    s.DrawRobotGL(i,q);
}

template <>
void DrawRoadmapNode(Geometric2DCSpace& s,const Config& q,int type)
{
  DrawConfiguration(s,q,type);
}

template <>
void DrawRoadmapNode(RigidRobot2DCSpace& s,const Config& q,int type)
{
  glBegin(GL_POINTS);
  glVertex2d(q(0),q(1));
  glEnd();
}

template <>
void DrawRoadmapNode(TranslatingRobot2DCSpace& s,const Config& q,int type)
{
  glBegin(GL_POINTS);
  glVertex2d(q(0),q(1));
  glEnd();
}

template <>
void DrawRoadmapNode(MultiRobot2DCSpace& s,const Config& q,int type)
{
  glBegin(GL_POINTS);
  for(size_t i=0;i<s.robots.size();i++) {
    Real u=Real(i)/Real(s.robots.size()-1);
    if(type == 1)
      glColor3f(u*0.5,u*0.5,1-u*0.5);
    else
      glColor3f(0.5+u*0.25,0.5+u*0.25,1-u*0.25);
    glVertex2d(q(0),q(1));
  }
}

template <>
void DrawRoadmapEdge(Geometric2DCSpace& s,const Config& a,const Config& b,int type)
{
  glBegin(GL_LINES);
  glVertex2d(a(0),a(1));
  glVertex2d(b(0),b(1));
  glEnd();
}

template <>
void DrawRoadmapEdge(RigidRobot2DCSpace& s,const Config& a,const Config& b,int type)
{
  glBegin(GL_LINES);
  glVertex2d(a(0),a(1));
  glVertex2d(b(0),b(1));
  glEnd();
}

template <>
void DrawRoadmapEdge(TranslatingRobot2DCSpace& s,const Config& a,const Config& b,int type)
{
  glBegin(GL_LINES);
  glVertex2d(a(0),a(1));
  glVertex2d(b(0),b(1));
  glEnd();
}

template <>
void DrawRoadmapEdge(MultiRobot2DCSpace& s,const Config& a,const Config& b,int type)
{
  glBegin(GL_LINES);
  for(size_t i=0;i<s.robots.size();i++) {
    Real u=Real(i)/Real(s.robots.size()-1);
    if(type == 1)
      glColor3f(u*0.5,u*0.5,1-u*0.5);
    else
      glColor3f(0.5+u*0.25,0.5+u*0.25,1-u*0.25);
    glVertex2v(s.GetRobotTransform(i,a).t);
    glVertex2v(s.GetRobotTransform(i,b).t);
  }
  glEnd();
}








bool DrawableCSpace::ReadFromXml(TiXmlElement* e)
{
  if(0==strcmp(e->Value(),"point2d_cspace")) {
    pointSpace=new Geometric2DCSpace;
    if(!XmlParse(e,*pointSpace)) {
      pointSpace = NULL;
      return false;
    }
  }
  else if(0==strcmp(e->Value(),"translating2d_cspace")) {
    translatingSpace = new TranslatingRobot2DCSpace;
    if(!XmlParse(e,*translatingSpace)) {
      translatingSpace = NULL;
      return false;
    }
  }
  else if(0==strcmp(e->Value(),"rigid2d_cspace")) {
    rigidSpace = new RigidRobot2DCSpace;
    if(!XmlParse(e,*rigidSpace)) {
      rigidSpace = NULL;
      return false;
    }
  }
  else if(0==strcmp(e->Value(),"multirobot2d_cspace")) {
    multiSpace = new MultiRobot2DCSpace;
    if(!XmlParse(e,*multiSpace)) {
      multiSpace = NULL;
      return false;
    }
  }
  else {
    return false;
  }
  queries.resize(0);
  TiXmlElement* q = e->FirstChildElement("planning_query");
  while(q) {
    queries.resize(queries.size()+1);
    if(!queries.back().ReadFromXml(q)) {
      queries.resize(queries.size()-1);
      return false;
    }
    q = q->NextSiblingElement("planning_query");
  }
  return true;
}

DrawableCSpace::operator CSpace* ()
{
  if(pointSpace) return &(*pointSpace);
  if(translatingSpace) return &(*translatingSpace);
  if(rigidSpace) return &(*rigidSpace);
  if(multiSpace) return &(*multiSpace);
  return NULL;
}

CSpace* DrawableCSpace::operator -> ()
{
  if(pointSpace) return &(*pointSpace);
  if(translatingSpace) return &(*translatingSpace);
  if(rigidSpace) return &(*rigidSpace);
  if(multiSpace) return &(*multiSpace);
  return NULL;
}

void DrawableCSpace::DrawStatic()
{
  if(pointSpace) ::DrawStatic(*pointSpace);
  if(translatingSpace) ::DrawStatic(*translatingSpace);
  if(rigidSpace) ::DrawStatic(*rigidSpace);
  if(multiSpace) ::DrawStatic(*multiSpace);
}

void DrawableCSpace::DrawConfiguration(const Config& q,int type)
{
  if(pointSpace) ::DrawConfiguration(*pointSpace,q,type);
  if(translatingSpace) ::DrawConfiguration(*translatingSpace,q,type);
  if(rigidSpace) ::DrawConfiguration(*rigidSpace,q,type);
  if(multiSpace) ::DrawConfiguration(*multiSpace,q,type);
}

//draw a small icon indicating a roadmap configuration
void DrawableCSpace::DrawRoadmapNode(const Config& q,int type)
{
  if(pointSpace) ::DrawRoadmapNode(*pointSpace,q,type);
  if(translatingSpace) ::DrawRoadmapNode(*translatingSpace,q,type);
  if(rigidSpace) ::DrawRoadmapNode(*rigidSpace,q,type);
  if(multiSpace) ::DrawRoadmapNode(*multiSpace,q,type);
}

void DrawableCSpace::DrawRoadmapEdge(const Config& a,const Config& b,int type)
{
  if(pointSpace) ::DrawRoadmapEdge(*pointSpace,a,b,type);
  if(translatingSpace) ::DrawRoadmapEdge(*translatingSpace,a,b,type);
  if(rigidSpace) ::DrawRoadmapEdge(*rigidSpace,a,b,type);
  if(multiSpace) ::DrawRoadmapEdge(*multiSpace,a,b,type);
}

#endif

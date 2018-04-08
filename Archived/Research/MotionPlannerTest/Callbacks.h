#ifndef DRAW_CALLBACKS_H
#define DRAW_CALLBACKS_H

#include <KrisLibrary/planning/MultiModalPlanner.h>
#include <KrisLibrary/planning/MotionPlanner.h>
#include <KrisLibrary/planning/KinodynamicMotionPlanner.h>
#include "DrawableSpace.h"
#include <KrisLibrary/planning/MCRPlanner.h>
#include <KrisLibrary/planning/MCRPlannerGoalSet.h>
#include <KrisLibrary/planning/DisplacementPlanner.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <KrisLibrary/GLdraw/ColorGradient.h>
#include <KrisLibrary/math/angle.h>

struct DrawTreeCallback : public TreeRoadmapPlanner::Node::Callback
{
  DrawTreeCallback(DrawableCSpace* _space=NULL)
    :space(_space),nodeColor(0,0,0),edgeColor(0,0,1)
  {}

  bool ForwardEdge(TreeRoadmapPlanner::Node* i,TreeRoadmapPlanner::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const SmartPointer<EdgePlanner>& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    if(space) {
      space->DrawRoadmapEdge(i->x,j->x);
    }
    else {
      glBegin(GL_LINES);
      glVertex2d(i->x(0),i->x(1));
      glVertex2d(j->x(0),j->x(1));
      glEnd();
    }
    return true;
  }

  void Visit(TreeRoadmapPlanner::Node* node)
  {
    nodeColor.setCurrentGL();
    if(space) {
      space->DrawRoadmapNode(node->x);
    }
    else {
      glBegin(GL_POINTS);
      glVertex2d(node->x(0),node->x(1));
      glEnd();
    }
  }

  DrawableCSpace* space;
  GLColor nodeColor,edgeColor;
};



struct DrawSBLPRTCallback : public Graph::CallbackBase<int>
{
  DrawSBLPRTCallback(SBLPRT* _prt,DrawableCSpace* _space=NULL)
    :space(_space),prt(_prt),nodeColor(0,0,0),edgeColor(0,0,1)
  {}

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    if(prt->IsEdgeConnected(i,j)) {
      glLineWidth(2.0);
      MilestonePath* path = prt->roadmap.FindEdge(i,j);
      if(space) {
	for(size_t i=0;i<path->edges.size();i++) {
	  space->DrawRoadmapEdge(path->edges[i]->Start(),path->edges[i]->End());
	}
      }
      else {
	glBegin(GL_LINE_STRIP);
	for(size_t i=0;i<path->edges.size();i++) {
	  glVertex2v(path->edges[i]->Start());
	  glVertex2v(path->edges[i]->End());
	}
	glEnd();
      }
    }
    else {
      glLineWidth(1.0);
      glBegin(GL_LINES);
      glVertex2v(*prt->roadmap.nodes[i]->root);
      glVertex2v(*prt->roadmap.nodes[j]->root);
      glEnd();
    }
    return true;
  }

  void Visit(int i)
  {
    nodeColor.setCurrentGL();
    if(space) {
      space->DrawRoadmapNode(*prt->roadmap.nodes[i]->root);
    }
    else {
      glBegin(GL_POINTS);
      glVertex2v(*prt->roadmap.nodes[i]->root);
      glEnd();
    }
  }

  DrawableCSpace* space;
  SBLPRT* prt;
  GLColor nodeColor,edgeColor;
};

struct DrawSBLTreeCallback : public SBLTree::Node::Callback
{
  DrawSBLTreeCallback(DrawableCSpace* _space=NULL)
    :space(_space),nodeColor(0,0,0),edgeColor(0,0,1)
  {}

  bool ForwardEdge(SBLTree::Node* i,SBLTree::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const SmartPointer<EdgePlanner>& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    if(space) {
      space->DrawRoadmapEdge(*i,*j);
    }
    else {
      glBegin(GL_LINES);
      glVertex2v(*i);
      glVertex2v(*j);
      glEnd();
    }
    return true;
  }

  void Visit(SBLTree::Node* node)
  {
    nodeColor.setCurrentGL();
    if(space)
      space->DrawRoadmapNode(*node);
    else {
      glBegin(GL_POINTS);
      glVertex2v(*node);
      glEnd();
    }
  }

  DrawableCSpace* space;
  GLColor nodeColor,edgeColor;
};

struct DrawGraphCallback : public Graph::CallbackBase<int>
{
  DrawGraphCallback(RoadmapPlanner::Roadmap& _prm,DrawableCSpace* _space=NULL)
    :space(_space),prm(_prm),doLazy(false)
  {}

  void Draw() {
    if(space) {
      edgeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) {
	RoadmapPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  if(doLazy) {
	    if((*e)->Done())
	      edgeColor.setCurrentGL();
	    else
	      lazyEdgeColor.setCurrentGL();
	  }
	  space->DrawRoadmapEdge(prm.nodes[e.source()],prm.nodes[e.target()]);
	}
      }
      nodeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) 
	space->DrawRoadmapNode(prm.nodes[i]);
    }
    else {
      edgeColor.setCurrentGL();
      glBegin(GL_LINES);
      for(size_t i=0;i<prm.nodes.size();i++) {
	RoadmapPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  if(doLazy) {
	    if((*e)->Done())
	      edgeColor.setCurrentGL();
	    else
	      lazyEdgeColor.setCurrentGL();
	  }
	  glVertex2v(prm.nodes[e.source()]);
	  glVertex2v(prm.nodes[e.target()]);
	}
      }
      glEnd();
      nodeColor.setCurrentGL();
      glBegin(GL_POINTS);
      for(size_t i=0;i<prm.nodes.size();i++) 
	glVertex2v(prm.nodes[i]);
      glEnd();
    }
  }

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
    return true;
  }

  void BackEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
  }

  void CrossEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i]);
    glVertex2v(prm.nodes[j]);
    glEnd();
  }

  void Visit(int i)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(prm.nodes[i]);
    glEnd();
  }

  DrawableCSpace* space;
  RoadmapPlanner::Roadmap& prm;
  GLColor nodeColor,edgeColor;
  bool doLazy;
  GLColor lazyEdgeColor;
};


struct DrawMCRCallback : public Graph::CallbackBase<int>
{
  DrawMCRCallback(MCRPlanner& planner,DrawableCSpace* _space=NULL)
    :space(_space),prm(planner.roadmap),modeGraph(planner.modeGraph) {}

  void Draw() {
    if(space) {
      edgeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) {
	MCRPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  space->DrawRoadmapEdge(prm.nodes[e.source()].q,prm.nodes[e.target()].q);
	}
      }
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=modeGraph.nodes[prm.nodes[i].mode].minCost;
	if(c >= DBL_MAX)
	  glColor3f(1,1,1);
	else
	  nodeColor[int(c)%nodeColor.size()].setCurrentGL();
	space->DrawRoadmapNode(prm.nodes[i].q);
      }
    }
    else {
      edgeColor.setCurrentGL();
      glBegin(GL_LINES);
      for(size_t i=0;i<prm.nodes.size();i++) {
	MCRPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  glVertex2v(prm.nodes[e.source()].q);
	  glVertex2v(prm.nodes[e.target()].q);
	}
      }
      glEnd();
      glBegin(GL_POINTS);
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=modeGraph.nodes[prm.nodes[i].mode].minCost;
	if(c >= DBL_MAX)
	  glColor3f(1,1,1);
	else
	  nodeColor[int(c)%nodeColor.size()].setCurrentGL();
	glVertex2v(prm.nodes[i].q);
      }
      glEnd();
    }
  }

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
    return true;
  }

  void BackEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void CrossEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void Visit(int i)
  {
    double c=modeGraph.nodes[prm.nodes[i].mode].minCost;
    if(c >= DBL_MAX)
      glColor3f(1,1,1);
    else
      nodeColor[int(c)%nodeColor.size()].setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(prm.nodes[i].q);
    glEnd();
  }

  DrawableCSpace* space;
  MCRPlanner::Roadmap& prm;
  MCRPlanner::ModeGraph& modeGraph;
  vector<GLColor> nodeColor;
  GLColor edgeColor;
};

struct DrawMCRGoalSetCallback : public Graph::CallbackBase<int>
{
  DrawMCRGoalSetCallback(MCRPlannerGoalSet& planner,DrawableCSpace* _space=NULL)
    :space(_space),prm(planner.roadmap),modeGraph(planner.modeGraph) {}

  void Draw() {
    if(space) {
      edgeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) {
	MCRPlannerGoalSet::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  space->DrawRoadmapEdge(prm.nodes[e.source()].q,prm.nodes[e.target()].q);
	}
      }
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=modeGraph.nodes[prm.nodes[i].mode].minCost;
	if(c >= DBL_MAX)
	  glColor3f(1,1,1);
	else
	  nodeColor[int(c)%nodeColor.size()].setCurrentGL();
	space->DrawRoadmapNode(prm.nodes[i].q);
      }
    }
    else {
      edgeColor.setCurrentGL();
      glBegin(GL_LINES);
      for(size_t i=0;i<prm.nodes.size();i++) {
	MCRPlannerGoalSet::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  glVertex2v(prm.nodes[e.source()].q);
	  glVertex2v(prm.nodes[e.target()].q);
	}
      }
      glEnd();
      glBegin(GL_POINTS);
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=modeGraph.nodes[prm.nodes[i].mode].minCost;
	if(c >= DBL_MAX)
	  glColor3f(1,1,1);
	else
	  nodeColor[int(c)%nodeColor.size()].setCurrentGL();
	glVertex2v(prm.nodes[i].q);
      }
      glEnd();
    }
  }

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
    return true;
  }

  void BackEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void CrossEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void Visit(int i)
  {
    double c=modeGraph.nodes[prm.nodes[i].mode].minCost;
    if(c >= DBL_MAX)
      glColor3f(1,1,1);
    else
      nodeColor[int(c)%nodeColor.size()].setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(prm.nodes[i].q);
    glEnd();
  }

  DrawableCSpace* space;
  MCRPlannerGoalSet::Roadmap& prm;
  MCRPlannerGoalSet::ModeGraph& modeGraph;
  vector<GLColor> nodeColor;
  GLColor edgeColor;
};


struct DrawDisplacementCallback : public Graph::CallbackBase<int>
{
  DrawDisplacementCallback(DisplacementPlanner& _planner,DrawableCSpace* _space=NULL)
    :space(_space),planner(_planner),prm(_planner.roadmap) 
  {
    drawReachableOnly = true;
    //rainbow
    nodeColor.params.resize(6);
    nodeColor.colors.resize(6);
    for(int i=0;i<6;i++) {
      nodeColor.params[i] = Real(i)/5;
      nodeColor.colors[i].setHSV(Real(i)/6*360,1,1);
    }
  }

  void Draw() {
    Real maxCost = 1;
    for(size_t i=0;i<prm.nodes.size();i++) {
      Real c=planner.OptimalCost(i);
      if(!IsInf(c) && c > maxCost) maxCost=c;
    }
    if(space) {
      edgeColor.setCurrentGL();
      for(size_t i=0;i<prm.nodes.size();i++) {
	if(drawReachableOnly && planner.pathCovers[i].covers.empty()) continue;
	DisplacementPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  if(drawReachableOnly && planner.pathCovers[e.target()].covers.empty()) continue;
	  space->DrawRoadmapEdge(prm.nodes[e.source()].q,prm.nodes[e.target()].q);
	}
      }
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=planner.OptimalCost(i)/maxCost;
	if(c > DBL_MAX) {
	  if(drawReachableOnly) continue;
	  if(!planner.IsCandidateForExploration(i)) continue;
	  glEnable(GL_BLEND);
	  glColor4f(0,0,0,0.25);
	}
	else {
	  GLColor col;
	  nodeColor.Eval(c,col);
	  col.setCurrentGL();
	}
	space->DrawRoadmapNode(prm.nodes[i].q);
      }
    }
    else {
      edgeColor.setCurrentGL();
      glBegin(GL_LINES);
      for(size_t i=0;i<prm.nodes.size();i++) {
	DisplacementPlanner::Roadmap::Iterator e;
	for(prm.Begin(i,e);!e.end();e++) {
	  if(drawReachableOnly && planner.pathCovers[e.target()].covers.empty()) continue;
	  glVertex2v(prm.nodes[e.source()].q);
	  glVertex2v(prm.nodes[e.target()].q);
	}
      }
      glEnd();
      glBegin(GL_POINTS);
      for(size_t i=0;i<prm.nodes.size();i++) {
	Real c=planner.OptimalCost(i)/maxCost;
	if(c > DBL_MAX) {
	  if(drawReachableOnly) continue;
	  if(!planner.IsCandidateForExploration(i)) continue;
	  glEnable(GL_BLEND);
	  glColor4f(0,0,0,0.25);
	}
	else {
	  GLColor col;
	  nodeColor.Eval(c,col);
	  col.setCurrentGL();
	}
	glVertex2v(prm.nodes[i].q);
      }
      glEnd();
    }
  }

  bool ForwardEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
    return true;
  }

  void BackEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void CrossEdge(int i,int j)
  {
    edgeColor.setCurrentGL();
    glBegin(GL_LINES);
    glVertex2v(prm.nodes[i].q);
    glVertex2v(prm.nodes[j].q);
    glEnd();
  }

  void Visit(int i)
  {
    Real c=planner.OptimalCost(i);
    if(c > DBL_MAX) {
      if(drawReachableOnly) return;
      if(!planner.IsCandidateForExploration(i)) return;
      glEnable(GL_BLEND);
      glColor4f(0,0,0,0.25);
    }
    else {
      GLColor col;
      nodeColor.Eval(c,col);
      col.setCurrentGL();
    }
    glBegin(GL_POINTS);
    glVertex2v(prm.nodes[i].q);
    glEnd();
  }

  DrawableCSpace* space;
  DisplacementPlanner& planner;
  DisplacementPlanner::Roadmap& prm;
  ColorGradient nodeColor;
  GLColor edgeColor;
  bool drawReachableOnly;
};

struct DrawCarCallback : public KinodynamicTree::Node::Callback
{
  void DrawPath(const KinodynamicMilestonePath& path) {
    glBegin(GL_LINE_STRIP);
    glVertex2v(path.milestones[0]);
    for(size_t k=1;k<path.milestones.size();k++) {
      Real d2 = Sqr(path.milestones[k][0] - path.milestones[k-1][0]) + Sqr(path.milestones[k][1] - path.milestones[k-1][1]);
      if(d2 < 0.02*0.02) 
        glVertex2v(path.milestones[k]);
      else {
        int n = (int)Ceil(Sqrt(d2) / 0.02);
        Vector temp;
        for(int j=0;j<n;j++) {
          Real u = Real(j+1)/Real(n);
          Real s = u*(path.paths[k-1]->ParamEnd()-path.paths[k-1]->ParamStart()) + path.paths[k-1]->ParamStart();
          path.paths[k-1]->Eval(s,temp);
          glVertex2v(temp);
        }
      }
    }
    glEnd();
  }
  bool ForwardEdge(KinodynamicTree::Node* i,KinodynamicTree::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const KinodynamicTree::EdgeData& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    DrawPath(e.path);
    return true;
  }

  void Visit(KinodynamicTree::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2v(*node);
    glEnd();
  }

  GLColor nodeColor,edgeColor;
};

struct DrawPendulumCallback : public KinodynamicTree::Node::Callback
{
  void DrawPath(const KinodynamicMilestonePath& path) {
    glBegin(GL_LINE_STRIP);
    for(size_t k=0;k<path.milestones.size();k++) 
      glVertex2f(AngleNormalize(path.milestones[k][0]+Pi)-Pi,path.milestones[k][1]);
    glEnd();
  }
  bool ForwardEdge(KinodynamicTree::Node* i,KinodynamicTree::Node* j)
  {
    //const Config& x=*i, &y=*j;
    const KinodynamicTree::EdgeData& e=j->edgeFromParent();
    edgeColor.setCurrentGL();
    DrawPath(e.path);
    return true;
  }

  void Visit(KinodynamicTree::Node* node)
  {
    nodeColor.setCurrentGL();
    glBegin(GL_POINTS);
    glVertex2f(AngleNormalize((*node)[0]+Pi)-Pi,(*node)[1]);
    glEnd();
  }

  GLColor nodeColor,edgeColor;
};


#endif

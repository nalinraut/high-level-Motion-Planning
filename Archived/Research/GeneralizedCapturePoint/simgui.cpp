#include "gcp.h"
#include <stdlib.h>
#include <stdio.h>
#include <KrisLibrary/Timer.h>
#include <KrisLibrary/math/random.h>
#include <KrisLibrary/utils/SmartPointer.h>
#include <KrisLibrary/GLdraw/GLUTNavigationProgram.h>
#include <KrisLibrary/GLdraw/GL.h>
#include <GL/glut.h>
#include <KrisLibrary/GLdraw/drawextra.h>
#include <Modeling/World.h>
#include <Slice.h>

//kg
const static Real defaultBodyMass = 1;
//m/s^2
const static Real defaultGravity = 9.8;
//m
const static Real defaultMinLegLength = 0.7;
//m
const static Real defaultMaxLegLength = 1.5;

//swing leg velocity bound
//inf: instantaneous switching of feet
//non-inf: max velocity, in m/s
const static Real defaultSwingLegVelocity = Inf;
//const static Real defaultSwingLegVelocity = 10;

//max acceleration produced by internal body forcing
const static Real maxCOMAcceleration = 0.1;

//random perturbations settings
//Average frequency, in Hz
const static Real perturbationFrequency = 0;
//Real perturbationFrequency = 7;
//Magnitude in Newtons
const static Real perturbationMagnitude = 5;


class LIPMState
{
public:
  Math3D::Vector3 bodyPosition,bodyVelocity;
  Math3D::Vector3 footPositions[2];
  int stanceFoot;
  bool swinging;
  Math3D::Vector3 swingFootDesired;
  Real ttotal;
  Real touchdownTime;

  LIPMState()
    :bodyPosition(0,0,1),bodyVelocity(0,0,0),stanceFoot(0),swinging(false),swingFootDesired(0,0,0),ttotal(0),touchdownTime(0)
  {
    footPositions[0].setZero();
    footPositions[1].setZero();
  }
  const Math3D::Vector3& StanceFootPosition() const { return footPositions[stanceFoot]; }
  const Math3D::Vector3& SwingFootPosition() const { return footPositions[1-stanceFoot]; }
  void SwitchStanceFoot() { stanceFoot = 1-stanceFoot; }
};

class LIPMControl
{
public:
  bool null;
  Math3D::Vector3 stanceFootDesired;
  Math3D::Vector3 comAcceleration;

  LIPMControl():null(true),comAcceleration(0.0) {}
  LIPMControl(const Math3D::Vector3& _nextFootPos, const Math3D::Vector3&_comAcceleration)
    :null(false),stanceFootDesired(_nextFootPos),comAcceleration(_comAcceleration)
  {}
};

class LIPM
{
public:
  Real m,g,maxLegLength,swingLegVelocity;
  LIPM() {
    m = defaultBodyMass;
    g = defaultGravity;
    maxLegLength = defaultMaxLegLength;
    swingLegVelocity = defaultSwingLegVelocity;
  }
   
  void NextState(const LIPMState& state,Real dt,const LIPMControl& control,LIPMState& xnext)
  {
    xnext = state;
    xnext.ttotal += dt;
    if(!control.null)
      cout<<"Requesting stance foot: "<<control.stanceFootDesired<<", accel "<<control.comAcceleration<<endl;
    //else
    //cout<<"Requesting accel "<<control.comAcceleration<<endl;
    if(!control.null && control.stanceFootDesired != state.StanceFootPosition()) {
      //print "Moving (not)",xnext.stanceFoot,"over time to to",control.stanceFootDesired
      xnext.swingFootDesired = control.stanceFootDesired;
      xnext.swinging = true;
    }
    //advance swing foot
    if(xnext.swinging) {
      Math3D::Vector3 swingFootErr = xnext.swingFootDesired - state.SwingFootPosition();
      if (swingFootErr.norm() < swingLegVelocity*dt) {
	//landed, set the new stance foot
	xnext.SwitchStanceFoot();
	//simulate impact of stabilizing body
	xnext.bodyVelocity[2] = 0;
	cout<<"Swing foot "<<xnext.stanceFoot<<" landed"<<endl;
	xnext.footPositions[xnext.stanceFoot] = xnext.swingFootDesired;
	xnext.touchdownTime = xnext.ttotal;
	xnext.swinging = false;
      }
      else {
	//move foot forward
	Math3D::Vector3 newswing = xnext.SwingFootPosition() + swingFootErr*(swingLegVelocity*dt/swingFootErr.norm());
	//limit swing foot trajectory by maximum leg length
	if(newswing.distance(xnext.bodyPosition) > maxLegLength) {
	  Math3D::Vector3 d = newswing - xnext.bodyPosition;
	  newswing = xnext.bodyPosition + d*(maxLegLength/d.norm());
	}
	xnext.footPositions[1-xnext.stanceFoot] = newswing;
      }
    }
    Math3D::Vector3 comp = xnext.bodyPosition;
    Math3D::Vector3 comv = xnext.bodyVelocity;
    Math3D::Vector3 xs = xnext.StanceFootPosition();
    if (comp[2] <= xs[2]) {
      //below ground plane, the thing must have fallen
      //print "Fallen!"
      return;
    }
    //compute support force and advance dynamics
    Vector3 a= control.comAcceleration;
    if(a.norm() > maxCOMAcceleration)
      a *= maxCOMAcceleration/a.norm();
    Vector3 hv=comv,ha=a;
    hv.z = 0;
    ha.z = 0;
    Real dz = 0, ddzdx2 = 0;
    if(hv.norm() > Epsilon)
      dz = comv.z/hv.norm();
    ddzdx2 = (control.comAcceleration.z - dz*ha.norm());
    Vector3 hcp_to_cm = comp - xs;
    hcp_to_cm.z = 0;
    Real f_over_m = (g + ddzdx2)/(comp.z - xs.z - dz*hcp_to_cm.norm());
    a.z = 0;
    Vector3 coma = f_over_m*(comp-xs) - Vector3(0,0,g) + a;

    //limit the COM position by the max leg length
    if(comp.distance(xs) > maxLegLength && comv.dot(comp-xs) > 0) {
      //fall along an arc
      Math3D::Vector3 d = comp-xs;
      //project position and velocity onto sphere
      comp = xs + d*(maxLegLength/d.norm());
      comv = comv - d*(comv.dot(d)/d.dot(d));
    }
    else {
      //hack: specify z acceleration.  this helps when |hv| approaches 0?
      coma.z = control.comAcceleration.z;
      comv += coma*dt;
      comp += comv*dt;
    }
    xnext.bodyPosition = comp;
    xnext.bodyVelocity = comv;
  }
};

    
class LIPMSimulator
{
public:
  LIPM lipm;
  LIPMState state;
  RobotWorld world;
  Real ttotal;
  LIPMSimulator(const char* terrain) {
    world.LoadElement("data/body.obj");
    world.LoadElement("data/foot1.obj");
    world.LoadElement("data/foot2.obj");
    if(terrain == NULL)
      world.LoadElement("data/plane.tri");
    else {
      int index = world.LoadElement(terrain);
      if(index < 0) {
	printf("Unable to load terrain file %g\n",terrain);
	exit(1);
      }
      AABB3D bb = world.terrains[0]->geometry->GetAABB();
      Ray3D r;
      r.source = 0.5*(bb.bmin + bb.bmax);
      r.source.z = bb.bmax.z + 1.0;
      r.direction.set(0,0,-1);
      Real d;
      if(world.terrains[0]->geometry->RayCast(r,&d)) {
	Vector3 pt;
	r.eval(d,pt);
	state.footPositions[0] = state.footPositions[1] = pt;
	state.bodyPosition = state.footPositions[0] + Vector3(0,0,1);
      }
      else {
	printf("Ray cast didn't work?\n");
	Vector3 pt = (bb.bmin+bb.bmax)*0.5;
	state.footPositions[0] = state.footPositions[1] = pt;
	state.bodyPosition = state.footPositions[0] + Vector3(0,0,1);
      }
    }
    world.rigidObjects[0]->geometry.Appearance()->faceColor.set(1,0,0,1);
    world.rigidObjects[1]->geometry.Appearance()->faceColor.set(0,1,0,1);
    world.rigidObjects[2]->geometry.Appearance()->faceColor.set(0,1,1,1);
    ttotal = 0;
  }
  void Advance(Real dt,const LIPMControl& control)
  {
    LIPMState xnext;
    lipm.NextState(state,dt,control,xnext);
    state = xnext;
    ttotal += dt;
  }
  void Reset() {
    state = LIPMState();
    ttotal = 0;
  }
  void DrawGL() {
    const LIPMState& s=state;
    world.rigidObjects[0]->T.t.set(s.bodyPosition);
    world.rigidObjects[1]->T.t.set(s.footPositions[0]);
    world.rigidObjects[2]->T.t.set(s.footPositions[1]);
    world.DrawGL();
    glDisable(GL_LIGHTING);
    glLineWidth(3.0);
    glBegin(GL_LINES);
    if(s.stanceFoot == 0)
      glColor3f(1,0,0);
    else
      glColor3f(1,0.7,0.7);
    GLDraw::glVertex3v(s.bodyPosition);
    GLDraw::glVertex3v(s.footPositions[0]);
    if(s.stanceFoot == 1)
      glColor3f(1,0,0);
    else
      glColor3f(1,0.7,0.7);
    GLDraw::glVertex3v(s.bodyPosition);
    GLDraw::glVertex3v(s.footPositions[1]);
    glEnd();
    //draw desired position
    if (s.swinging) {
      glEnable(GL_LIGHTING);
      float col[]={1,0.5,0,1};
      glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE,col);
      GLDraw::drawBoundingBox(s.swingFootDesired+Math3D::Vector3(-0.02),
			      s.swingFootDesired+Math3D::Vector3(0.02));
    }
  }
};

class LIPMControllerBase
{
public:
  Vector3 goal;
  LIPMControllerBase() : goal(0,0,0) {}
  virtual void Reset() {}
  virtual LIPMControl Evaluate(LIPMState& state,Real ttotal) =0;
  virtual void DrawGL() {
    glDisable(GL_LIGHTING);
    glPointSize(5.0);
    glColor3f(1,1,1);
    glBegin(GL_POINTS);
    GLDraw::glVertex3v(goal);
    glEnd();
  }
};


class BodyPDController : public LIPMControllerBase
{
public:
  Real kP,kD;
  BodyPDController(Real _kP,Real _kD) : kP(_kP),kD(_kD) {}
  virtual LIPMControl Evaluate(LIPMState& state,Real ttotal) {
    Vector3 f = kP*(goal-state.bodyPosition) + kD*(-state.bodyVelocity);;
    return LIPMControl(state.StanceFootPosition(),f);
  }
};

class LinearCapturePointController : public LIPMControllerBase
{
public:
  bool walkMode;
  LinearCapturePointController():walkMode(true) {}
  virtual LIPMControl Evaluate(LIPMState& state,Real ttotal) {
    //Real kP = 10.0;
    //Real kD = 3.0;
    Real kP = 0;
    Real kD = 0;
    Vector3 f = kP*(goal-state.bodyPosition) + kD*(-state.bodyVelocity);

    if(ttotal > state.touchdownTime+0.5 || state.swinging) {
      Vector3 hv = state.bodyVelocity;
      if(walkMode) {
	Vector3 d = (goal - state.bodyPosition) - 0.3*state.bodyVelocity;
	if(d.norm() > 0.25) d *= 0.25/d.norm();
	hv -= d;
      }
      hv.z = 0;
      Vector3 xb = state.StanceFootPosition();
      Vector3 cp = state.bodyPosition + hv * Sqrt((state.bodyPosition.z - xb.z)/defaultGravity);
      cp.z = xb.z;
      return LIPMControl(cp,f);
    }
    else {
      LIPMControl u;
      u.comAcceleration = f;
      return u;
    }
  }
};

class GeneralizedCapturePointController : public LIPMControllerBase
{
public:
  bool walkMode;
  const Terrain& env;
  Meshing::TriMesh upwardFacingTrimesh;
  std::vector<Real> pathCurvatures;
  std::vector<GCP::Vector2> cps2d;
  std::vector<Vector3> cps3d;
  std::vector<Vector3> cmTrajectory;
  std::vector<Real> cmTrajectoryTimes;

  GeneralizedCapturePointController(const Terrain& _env):walkMode(true),env(_env) {
    const Meshing::TriMesh& mesh = env.geometry->AsTriangleMesh();
    for(size_t i=0;i<mesh.tris.size();i++) {
      if(mesh.TriangleNormal(i).z >= 0) {
	int start = upwardFacingTrimesh.verts.size();
	upwardFacingTrimesh.verts.push_back(mesh.verts[mesh.tris[i].a]);
	upwardFacingTrimesh.verts.push_back(mesh.verts[mesh.tris[i].b]);
	upwardFacingTrimesh.verts.push_back(mesh.verts[mesh.tris[i].c]);
	upwardFacingTrimesh.tris.push_back(IntTriple(start,start+1,start+2));
      }
    }
    printf("%d / %d triangles facing upward\n",mesh.tris.size(),upwardFacingTrimesh.tris.size());
  }
  virtual void Reset() { cmTrajectory.resize(0); cmTrajectoryTimes.resize(0); }
  virtual LIPMControl Evaluate(LIPMState& state,Real ttotal) {
    Real kP = 10.0;
    Real kD = 3.0;
    Vector3 goalVelocity(0.0);
    //regulate height according to predicted COM path
    //printf("Slope %g, curvature %g\n",slope,curvature);
    while(cmTrajectoryTimes.size() >= 2 && ttotal > cmTrajectoryTimes[1]) {
      cmTrajectoryTimes.erase(cmTrajectoryTimes.begin());
      cmTrajectory.erase(cmTrajectory.begin());
    }
    if(cmTrajectoryTimes.size()==1)
      goal.z = cmTrajectory[0].z;
    else if(cmTrajectoryTimes.size() >= 2) {
      Real u = (ttotal - cmTrajectoryTimes[0])/(cmTrajectoryTimes[1]-cmTrajectoryTimes[0]);
      goal.z = cmTrajectory[0].z + u*(cmTrajectory[1].z-cmTrajectory[0].z);
      goalVelocity.z = (cmTrajectory[1].z-cmTrajectory[0].z)/(cmTrajectoryTimes[1]-cmTrajectoryTimes[0]);
    }
    Vector3 xb = state.StanceFootPosition();
    if(goal.z < defaultMinLegLength+xb.z) goal.z = defaultMinLegLength+xb.z;
    if(goal.z > defaultMaxLegLength+xb.z) goal.z = defaultMaxLegLength+xb.z;
    Vector3 f = kP*(goal-state.bodyPosition) + kD*(goalVelocity-state.bodyVelocity);
    f.x = 0;
    f.y = 0;

    if(ttotal > state.touchdownTime+0.5 || state.swinging) {
      Vector3 hv = state.bodyVelocity;
      if(walkMode) {
	Vector3 d = (goal - state.bodyPosition) - 0.3*state.bodyVelocity;
	if(d.norm() > 0.25) d *= 0.25/d.norm();
	hv -= d;
      }
      hv.z = 0;
      if(hv.norm() < 1e-3) {
	LIPMControl u;
	u.comAcceleration = f;

	//save z path
	cmTrajectory.resize(0);
	cmTrajectoryTimes.resize(0);
	return u;
      }
      Math3D::Plane3D p;
      Vector3 zdirection = Vector3(0,0,1);
      Vector3 ydirection; ydirection.setCross(hv,zdirection); ydirection.inplaceNormalize();
      Vector3 xdirection = hv; xdirection.inplaceNormalize();
      p.setPointNormal(state.bodyPosition,ydirection);
      vector<Math3D::Segment3D> segments;
      //Geometry::Slice(env.geometry.TriangleMeshCollisionData(),p,segments);
      Geometry::Slice(upwardFacingTrimesh,p,segments);
      GCP::Problem problem;
      problem.Lmin = defaultMinLegLength;
      problem.Lmax = defaultMaxLegLength;
      problem.hmin = defaultMinLegLength;
      problem.hmax = defaultMaxLegLength;
      problem.minGs = 0.5;
      problem.maxGs = 1.5;
      //problem.SetInitialConditions(GCP::Vector2(xdirection.dot(state.bodyPosition),zdirection.dot(state.bodyPosition)),
      problem.SetInitialConditions(GCP::Vector2(xdirection.dot(state.bodyPosition),zdirection.dot(goal)),
				   GCP::Vector2(xdirection.dot(hv),state.bodyVelocity.z));
      problem.SetFriction(2.0);
      problem.terrain.segments.reserve(segments.size());
      for(size_t i=0;i<segments.size();i++) {
	GCP::Segment s;
	s.a.x = xdirection.dot(segments[i].a);
	s.a.y = zdirection.dot(segments[i].a);
	s.b.x = xdirection.dot(segments[i].b);
	s.b.y = zdirection.dot(segments[i].b);
	if(!IsFinite(s.a.x) || !IsFinite(s.b.x) || !IsFinite(s.b.x) || !IsFinite(s.b.y)) continue;
	s.Sort();
	problem.terrain.segments.push_back(s);
      }
      problem.SolveAllCurvatureCPs(5e-3,pathCurvatures,cps2d);
      if(problem.SolveCPFromCurvature(0)) {
	cps2d.push_back(problem.capturePoint);
	pathCurvatures.push_back(0);
      }
      else
	printf("Unable to solve straight line path\n");

      GCP::Vector2 cp2d;
      if(cps2d.empty()) {
	//do the best you can...
	problem.SolveCPFromCurvature(0);
	cp2d = problem.capturePoint;

	//save z path
	vector<GCP::Vector2> xtrace,vtrace;
	problem.SimulationTrace(0.01,100,xtrace,vtrace);
	cmTrajectoryTimes.resize(xtrace.size());
	cmTrajectory.resize(xtrace.size());
	for(size_t i=0;i<xtrace.size();i++) {
	  cmTrajectoryTimes[i] = ttotal + 0.01*i;
	  cmTrajectory[i] = ydirection*p.offset + xtrace[i].x*xdirection + xtrace[i].y*zdirection;
	}
      }
      else {
	//pick the best according to some criterion.  Here choose one with height closest to 1
	int best=0;
	Real err = Inf;
	for(size_t i=0;i<cps2d.size();i++) {
	  problem.pathCurvature = pathCurvatures[i];
	  Real z = problem.PathHeight(cps2d[i].x);
	  printf("  CP %g %g Resulting height: %g\n",cps2d[i].x,cps2d[i].y,z);
	  if(Abs(z - cps2d[i].y - 1) < err) {
	    best = (int)i;
	    err = Abs(z - cps2d[i].y - 1);
	  }
	}
	cp2d = cps2d[best];
	printf("Best cp %g %g, curvature: %g, resting height error %g\n",cp2d.x,cp2d.y,pathCurvatures[best],err);

	//save z path
	problem.pathCurvature = pathCurvatures[best];
	problem.capturePoint = cp2d;
	vector<GCP::Vector2> xtrace,vtrace;
	problem.SimulationTrace(0.01,100,xtrace,vtrace);
	cmTrajectoryTimes.resize(xtrace.size());
	cmTrajectory.resize(xtrace.size());
	for(size_t i=0;i<xtrace.size();i++) {
	  cmTrajectoryTimes[i] = ttotal + 0.01*i;
	  cmTrajectory[i] = ydirection*p.offset + xtrace[i].x*xdirection + xtrace[i].y*zdirection;
	}
      }
      //visualization: convert back to 3D
      cps3d.resize(cps2d.size());
      for(size_t i=0;i<cps2d.size();i++)
	cps3d[i] = ydirection*p.offset + xdirection*cps2d[i].x + zdirection*cps2d[i].y;
      //convert back to 3D
      Vector3 cp = ydirection*p.offset + cp2d.x*xdirection + cp2d.y*zdirection;
      return LIPMControl(cp,f);
    }
    else {
      LIPMControl u;
      u.comAcceleration = f;
      return u;
    }
  }
  virtual void DrawGL() {
    if(walkMode)
      LIPMControllerBase::DrawGL();
    glPointSize(5.0);
    glColor3f(0,1,0);
    glBegin(GL_POINTS);
    for(size_t i=0;i<cps3d.size();i++)
      GLDraw::glVertex3v(cps3d[i]);
    glEnd();
    //debug CM trajectory
    /*
    glLineWidth(4.0);
    glColor3f(1,0,0);
    glBegin(GL_LINE_STRIP);
    for(size_t i=0;i<cmTrajectory.size();i++)
      GLDraw::glVertex3v(cmTrajectory[i]);
    glEnd();
    */
  }
};


class GLTest : public GLUTNavigationProgram
{
public:
  LIPMSimulator sim;
  SmartPointer<LIPMControllerBase> controller;
  bool simulate;
  Real dt;
  bool perturbationMode;
  Timer timer;
  double lastIdleTime;
  GLTest(const char* fn):GLUTNavigationProgram(),sim(fn)
  {
    simulate = false;
    perturbationMode = false;
    dt = 0.01;
    lastIdleTime = 0;
    //controller = new LinearCapturePointController();
    controller = new GeneralizedCapturePointController(*sim.world.terrains[0]);
    //controller = new BodyPDController(10,3);
    controller->goal = sim.state.bodyPosition;
  }

  virtual bool Initialize() {
    if(!GLUTNavigationProgram::Initialize()) return false;
    sim.world.lights.resize(1);
    sim.world.lights[0].setColor(GLDraw::GLColor(1,1,1));
    sim.world.lights[0].setDirectionalLight(Math3D::Vector3(0.2,-0.4,1));
    sim.world.lights[0].setColor(GLDraw::GLColor(1,1,1));

    camera.dist = 6;
    camera.tgt = sim.state.bodyPosition;
    viewport.n = 0.1;
    viewport.f = 100;
    viewport.setLensAngle(DtoR(30.0));
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
    glClearColor(sim.world.background.rgba[0],sim.world.background.rgba[1],sim.world.background.rgba[2],sim.world.background.rgba[3]);
    glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    return true;
  }

  virtual void RenderWorld() {
    glEnable(GL_LIGHTING);
    sim.world.SetGLLights();
    sim.DrawGL();
    if(controller) controller->DrawGL();
    glLineWidth(1.0);
  }

  virtual void RenderScreen()
  {
  }
        
  void step_simulation(Real dt) {
    LIPMControl control;
    if(controller) control = controller->Evaluate(sim.state,sim.ttotal);
    apply_external_forces(dt);
    sim.Advance(dt,control);
  }

  void apply_external_forces(Real dt) {
    //apply perturbation if activated
    //exponential distribution
    if(Rand() > Exp(-dt*perturbationFrequency)) {
      Math3D::Vector3 f(Rand(-perturbationMagnitude,perturbationMagnitude),
			Rand(-perturbationMagnitude,perturbationMagnitude),
			Rand(-perturbationMagnitude,perturbationMagnitude));
      sim.state.bodyVelocity += f*(dt/sim.lipm.m);
    }
  }

  virtual void Handle_Keypress(unsigned char c,int x, int y)
  {
    switch(c){
    case ' ':
      step_simulation(dt);
      Refresh();
      break;
    case 's':
      simulate = !simulate;
      if(simulate) printf("Switched simulation on\n");
      else printf("Switched simulation off\n");
      break;
    case 'p':
      perturbationMode = !perturbationMode;
      if(perturbationMode) printf("Switched to perturbation mode\n");
      else printf("Switched to goal mode\n");
      if(dynamic_cast<LinearCapturePointController*>(&(*controller)) != NULL) 
	dynamic_cast<LinearCapturePointController*>(&(*controller))->walkMode = !perturbationMode;
      if(dynamic_cast<GeneralizedCapturePointController*>(&(*controller)) != NULL) 
	dynamic_cast<GeneralizedCapturePointController*>(&(*controller))->walkMode = !perturbationMode;
      break;
    case 'r':
      printf("Resetting simulation\n");
      sim.Reset();
      if(controller) controller->Reset();
      Refresh();
      break;
    case 'h':
      printf("Help: \
'h': prints this message \
[space]: steps the simulation \
's': toggles simulation \
'r': resets simulation \
[up/down/left/right]: controls the goal position \
");
      break;
    }
  }
  void Handle_Special(int c,int x,int y)
  {
    switch(c) {
    case GLUT_KEY_UP:
      if(perturbationMode) sim.state.bodyVelocity += Vector3(0,0.25,0);
      else { if(controller) controller->goal[1] += 0.5; }
      break;
    case GLUT_KEY_DOWN:
      if(perturbationMode) sim.state.bodyVelocity += Vector3(0,-0.25,0);
      else { if(controller) controller->goal[1] -= 0.5; }
      break;
    case GLUT_KEY_RIGHT:
      if(perturbationMode) sim.state.bodyVelocity += Vector3(0.25,0,0);
      else { if(controller) controller->goal[0] += 0.5; }
      break;
    case GLUT_KEY_LEFT:
      if(perturbationMode) sim.state.bodyVelocity += Vector3(-0.25,0,0);
      else { if(controller) controller->goal[0] -= 0.5; }
      break;
    }
    Refresh();
  }

  void Handle_Idle()
  {
    GLUTNavigationProgram::Handle_Idle();
    if(simulate) {
      double t = timer.ElapsedTime();
      while(t > lastIdleTime) {
	step_simulation(dt);
	t -= dt;
      }
      Refresh();
    }
    double t = timer.ElapsedTime();
    SleepIdleCallback(Max(0.0,dt-(t-lastIdleTime))*1000);
    lastIdleTime = t;
  }
};

int main(int argc,const char** argv)
{
  const char* fn = NULL;
  if(argc > 1) fn = argv[1];
  GLTest program(fn);
  program.Run("Simulation GUI");
}

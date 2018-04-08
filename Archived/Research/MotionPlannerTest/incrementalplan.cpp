#include <KrisLibrary/planning/IncrementalMotionPlanner.h>
#include <KrisLibrary/planning/PointLocation.h>
#include <KrisLibrary/math3d/interpolate.h>
#include <KrisLibrary/Modeling/World.h>
#include <KrisLibrary/Modeling/Interpolate.h>
#include <fstream>

class IncrementalWorldCSpace : public IncrementalCollisionCSpace
{
public:
  IncrementalWorldCSpace(RobotWorld& world);
  virtual void ConfigToTransforms(const Config& q);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { Interpolate(x,y,0.5,out); }

  RobotWorld& world;
  vector<vector<int> > robotToMesh;
  vector<int> rigidObjectToMesh;
  vector<int> terrainToMesh;
};

Vector stack(const Vector& a, const Vector& b)
{
  Vector out(a.n+b.n);
  if(!a.empty()) out.copySubVector(0,a);
  if(!b.empty()) out.copySubVector(a.n,b);
  return out;
}

IncrementalWorldCSpace::IncrementalWorldCSpace(RobotWorld& _world)
  :world(_world)
{
  robotToMesh.resize(world.robots.size());
  terrainToMesh.resize(world.terrains.size(),-1);
  for(size_t i=0;i<world.robots.size();i++) {
    bMin = stack(bMin,world.robots[i].robot->qMin);
    bMax = stack(bMax,world.robots[i].robot->qMax);
    robotToMesh[i].resize(world.robots[i].robot->geometry.size(),-1);
    for(size_t j=0;j<world.robots[i].robot->geometry.size();j++) {
      if(world.robots[i].robot->geometry[j].type == Geometry::AnyGeometry3D::TriangleMesh)
	robotToMesh[i][j] = AddMesh(world.robots[i].robot->geometry[j].AsTriangleMesh(),world.robots[i].robot->LinkName(j).c_str());
    }
  }
  //TODO: rigid objects
  for(size_t i=0;i<world.terrains.size();i++) {
    if(world.terrains[i].terrain->geometry.type == Geometry::AnyGeometry3D::TriangleMesh)
      terrainToMesh[i] = AddMesh(world.terrains[i].terrain->geometry.AsTriangleMesh(),world.terrains[i].name.c_str());
  }
  for(size_t i=0;i<world.robots.size();i++) {
    for(size_t j=0;j<world.robots[i].robot->geometry.size();j++) {
      if(robotToMesh[i][j] < 0) continue;
      //add self collisions
      for(size_t k=0;k<world.robots[i].robot->geometry.size();k++) {
	if(robotToMesh[i][k] < 0) continue;
	if(world.robots[i].robot->selfCollisions(j,k) != NULL) 
	  AddCollision(robotToMesh[i][j],robotToMesh[i][k]);
      }

      //TODO: add robot to robot collisions

      //add robot to terrain collisions
      for(size_t k=0;k<world.terrains.size();k++) {
	if(terrainToMesh[k] < 0) continue;
	if(world.robots[i].robot->parents[j] < 0) continue;
	AddCollision(robotToMesh[i][j],terrainToMesh[k]);
      }
    }
  }
}

void IncrementalWorldCSpace::ConfigToTransforms(const Config& q)
{
  int n=0;
  for(size_t i=0;i<world.robots.size();i++) {
    Config qi;
    qi.setRef(q,n,1,world.robots[i].robot->q.n);
    world.robots[i].robot->UpdateConfig(qi);
    n += qi.n;
  }
  Assert(n==q.n);
  for(size_t i=0;i<robotToMesh.size();i++) {
    for(size_t j=0;j<robotToMesh[i].size();j++)
      if(robotToMesh[i][j] >= 0) 
	meshes[robotToMesh[i][j]]->UpdateTransform(world.robots[i].robot->links[j].T_World);
  }
}

Real IncrementalWorldCSpace::Distance(const Config& x, const Config& y)
{
  Real d=0;
  int n=0;
  for(size_t i=0;i<world.robots.size();i++) {
    Config xi,yi;
    xi.setRef(x,n,1,world.robots[i].robot->q.n);
    yi.setRef(y,n,1,world.robots[i].robot->q.n);
    d += ::Distance(*world.robots[i].robot,xi,yi,2);
    n += xi.n;
  }
  Assert(n==x.n);
  return d;
}

void IncrementalWorldCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  out.resize(x.n);
  int n=0;
  for(size_t i=0;i<world.robots.size();i++) {
    Config xi,yi,outi;
    xi.setRef(x,n,1,world.robots[i].robot->q.n);
    yi.setRef(y,n,1,world.robots[i].robot->q.n);
    outi.setRef(out,n,1,world.robots[i].robot->q.n);
    ::Interpolate(*world.robots[i].robot,xi,yi,u,outi);
    n += xi.n;
  }
  Assert(n==x.n);
}


class IncrementalRigidCSpace : public IncrementalCollisionCSpace
{
public:
  IncrementalRigidCSpace();
  virtual void ConfigToTransforms(const Config& q);
  virtual Real Distance(const Config& x, const Config& y);
  virtual void Interpolate(const Config& x,const Config& y,Real u,Config& out);
  virtual void Midpoint(const Config& x,const Config& y,Config& out) { Interpolate(x,y,0.5,out); }

  bool LoadRobot(const char* fn);
  bool LoadObstacle(const char* fn);

  vector<int> robotMeshes,obstacleMeshes;
};

IncrementalRigidCSpace::IncrementalRigidCSpace()
{}

RigidTransform ConfigToTransform(const Config& q)
{
  RigidTransform T;
  T.t.set(q[0],q[1],q[2]);
  EulerAngleRotation ea(q[3],q[4],q[5]);
  ea.getMatrixZYX(T.R);
  return T;
}

void TransformToConfig(const RigidTransform& T,Config& q)
{
  T.t.get(q[0],q[1],q[2]);
  EulerAngleRotation ea;
  ea.setMatrixZYX(T.R);
  ea.get(q[3],q[4],q[5]);
}

void IncrementalRigidCSpace::ConfigToTransforms(const Config& q)
{
  Assert(q.n == 6*(int)robotMeshes.size());
  for(size_t i=0;i<robotMeshes.size();i++) {
    Vector qi;
    qi.setRef(q,i*6,1,6);
    RigidTransform T=ConfigToTransform(qi);
    meshes[robotMeshes[i]]->UpdateTransform(T);
  }
}

Real IncrementalRigidCSpace::Distance(const Config& x, const Config& y)
{
  Assert(x.n == 6*(int)robotMeshes.size());
  Assert(y.n == 6*(int)robotMeshes.size());
  Real d = 0;
  for(size_t i=0;i<robotMeshes.size();i++) {
    Vector xi,yi;
    xi.setRef(x,i*6,1,6);
    yi.setRef(y,i*6,1,6);
    RigidTransform Tx=ConfigToTransform(xi);
    RigidTransform Ty=ConfigToTransform(yi);
    d += Tx.t.distanceSquared(Ty.t);
    Matrix3 mdiff;
    mdiff.mulTransposeB(Tx.R,Ty.R);
    AngleAxisRotation aa; aa.setMatrix(mdiff);
    d += Sqr(aa.angle);
  }
  return Sqrt(d);
}

void IncrementalRigidCSpace::Interpolate(const Config& x,const Config& y,Real u,Config& out)
{
  Assert(x.n == 6*(int)robotMeshes.size());
  Assert(y.n == 6*(int)robotMeshes.size());
  out.resize(x.n);
  for(size_t i=0;i<robotMeshes.size();i++) {
    Vector xi,yi,outi;
    xi.setRef(x,i*6,1,6);
    yi.setRef(y,i*6,1,6);
    outi.setRef(out,i*6,1,6);
    RigidTransform Tx=ConfigToTransform(xi);
    RigidTransform Ty=ConfigToTransform(yi);
    RigidTransform Tout;
    interpolate(Tx,Ty,u,Tout);
    TransformToConfig(Tout,outi);
  }
}

bool IncrementalRigidCSpace::LoadRobot(const char* fn)
{
  int m = AddMesh(fn);
  if(m < 0) return false;
  robotMeshes.push_back(m);
  for(size_t i=0;i+1<robotMeshes.size();i++)
    AddCollision(robotMeshes[i],m);
  for(size_t i=0;i<obstacleMeshes.size();i++)
    AddCollision(obstacleMeshes[i],m);
  Vector newbmin(bMin.n+6),newbmax(bMax.n+6);
  if(!bMin.empty()) {
    newbmin.copySubVector(0,bMin);
    newbmax.copySubVector(0,bMax);
  }
  for(int i=0;i<3;i++) {
    newbmin[bMax.n+i] = -1;
    newbmax[bMax.n+i] = 1;
    newbmin[bMax.n+3+i] = -Pi;
    newbmax[bMax.n+3+i] = Pi;
  }
  return true;
}

bool IncrementalRigidCSpace::LoadObstacle(const char* fn)
{
  int m = AddMesh(fn);
  if(m < 0) return false;
  obstacleMeshes.push_back(m);
  for(size_t i=0;i<robotMeshes.size();i++)
    AddCollision(m,robotMeshes[i]);
  return true;
}


bool Plan(IncrementalCollisionCSpace* space,const Config& a,const Config& b,
	  int iterLimit,Real timeLimit,const char* outFn)
{
  IncrementalRRTPlanner planner(space);
  planner.Init(a,b);
  bool res = planner.Plan(iterLimit,timeLimit);
  if(res) {
    MilestonePath path;
    planner.CreatePath(0,1,path);
    ofstream out(outFn);
    if(!out) {
      printf("Error opening output file %s\n",outFn);
      return false;
    }
    for(size_t i=0;i<=path.edges.size();i++)
      out<<i<<"\t"<<path.GetMilestone(i)<<endl;
    out.close();
    return true;
  }
  printf("Planning failed\n");
  return false;
}

bool DoPlanning(const char* worldfile,const char* configsfile)
{
  RobotWorld world;
  if(!world.LoadXML(worldfile)) {
    printf("Error opening world file %s\n",worldfile);
    return false;
  }

  vector<Config> configs;
  ifstream in(configsfile);
  if(!in) {
    printf("Error opening configs file %s\n",configsfile);
    return false;
  }
  while(in) {
    Config temp;
    in >> temp;
    if(in) configs.push_back(temp);
  }
  if(configs.size() < 2) {
    printf("Configs file does not contain 2 or more configs\n");
    return false;
  }
  Config start=configs[0];
  Config goal=configs[1];
  
  IncrementalWorldCSpace cspace(world);
  if(start.n != cspace.bMin.n) {
    printf("Incorrect size on start configuration\n");
    return false;
  }
  if(goal.n != cspace.bMin.n) {
    printf("Incorrect size on goal configuration\n");
    return false;
  }
  if(!cspace.IsFeasible(start)) {
    printf("Infeasible start configuration\n");
    return false;
  }
  if(!cspace.IsFeasible(goal)) {
    printf("Infeasible goal configuration\n");
    return false;
  }
  Plan(&cspace,start,goal,10000,20,"incrementalplan.path");
  return true;
}

int main(int argc,const char** argv)
{
  const char* worldfile = argv[1];
  const char* configsfile = argv[2];

  if(!DoPlanning(worldfile,configsfile)) return 1;
  return 0;
}

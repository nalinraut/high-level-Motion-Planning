#include "KlamptCollide.h"
#include <statistics/KMeans.h>
#include <statistics/HierarchicalClustering.h>
#include <utils/permutation.h>
#include <Timer.h>
#include <errors.h>
using namespace std;

namespace KlamptCollide {


//limits on the size of the distance matrix allowed for kmeans clustering (#contacts*k). 
const static size_t gMaxKMeansSize = 5000;
//limits on the size of the distance matrix allowed for hierarchical clustering (#contacts^2).
const static size_t gMaxHClusterSize = 2000;
//true if random subsampling should be used in case of excessinve numbers of contact
const static bool gSubsampleRandomInit = false;

//true if random initialization should be used
const static bool gKmeansRandomInit = false;
//number of kmeans iterations
const static int gKmeansIters = 20;

//if a normal has this length then it is ignored
const static Real gZeroNormalTolerance = 1e-4;


//if two contact points are closer than this threshold, will try to look
//at the local geometry to derive a contact normal
const static Real gNormalFromGeometryTolerance = 1e-5;
//const static Real gNormalFromGeometryTolerance = 1e-2;

//if a barycentric coordinate is within this tolerance of zero, it will be
//considered a zero
const static Real gBarycentricCoordZeroTolerance = 1e-3;

//if true, takes the ODE tolerance points and performs additional contact
//checking -- useful for flat contacts
const static bool gDoTriangleTriangleCollisionDetection = false;

//if gDoTriangleTriangleCollisionDetection is true, this doesn't consider unique contact points if they are between this tolerance
const static Real cptol=1e-5;




void ReverseContact(ContactPair& contact)
{
  std::swap(contact.id1,contact.id2);
  for(int k=0;k<3;k++) contact.normal[k]*=-1.0;
  std::swap(contact.element1,contact.element2);
}


BLEG::BLEG(int _id,Real _outerMargin)
  :id(_id),tempGeometry(NULL),geometry(NULL),geometryOwned(false),outerMargin(_outerMargin)
{
}

bool BLEG::LoadGeometry(const char* fn)
{
  ClearGeometry();
  tempGeometry = new AnyGeometry3D();
  if(!tempGeometry->Load(fn)) return false;
  geometryOwned = true;
  geometry = new AnyCollisionGeometry3D(*tempGeometry);
  Assert(geometry->type == AnyGeometry3D::TriangleMesh);
  AnyCast<CollisionMesh>(&geometry->collisionData)->CalcIncidentTris();
  AnyCast<CollisionMesh>(&geometry->collisionData)->CalcTriNeighbors();
  geometry->margin = outerMargin;
  return true;
}


void BLEG::SetGeometry(const GeometricPrimitive3D& geom)
{
  ClearGeometry();
  tempGeometry = new AnyGeometry3D(geom);
  geometryOwned = true;
  geometry = new AnyCollisionGeometry3D(*tempGeometry);
  geometry->margin = outerMargin;
}


void BLEG::SetGeometry(const vector<Vector3>& points)
{
  Meshing::PointCloud3D pc;
  pc.points = points;
  SetGeometry(pc);
}

void BLEG::SetGeometry(const Meshing::PointCloud3D& pointCloud)
{
  ClearGeometry();
  tempGeometry = new AnyGeometry3D(pointCloud);
  geometryOwned = true;
  geometry = new AnyCollisionGeometry3D(*tempGeometry);
  geometry->margin = outerMargin;
}

void BLEG::SetGeometry(const Meshing::TriMesh& mesh)
{
  ClearGeometry();
  tempGeometry = new AnyGeometry3D(mesh);
  geometryOwned = true;
  geometry = new AnyCollisionGeometry3D(*tempGeometry);
  Assert(geometry->type == AnyGeometry3D::TriangleMesh);
  AnyCast<CollisionMesh>(&geometry->collisionData)->CalcIncidentTris();
  AnyCast<CollisionMesh>(&geometry->collisionData)->CalcTriNeighbors();
  geometry->margin = outerMargin;
}

void BLEG::SetGeometry(AnyCollisionGeometry3D* _geometry)
{
  ClearGeometry();
  geometry = _geometry;
  geometryOwned = false;
  if(geometry->type == AnyGeometry3D::TriangleMesh) {
    AnyCast<CollisionMesh>(&geometry->collisionData)->CalcIncidentTris();
    AnyCast<CollisionMesh>(&geometry->collisionData)->CalcTriNeighbors();
  }
  geometry->margin = outerMargin;
}

BLEG::~BLEG()
{
  ClearGeometry();
}

void BLEG::SetMargin(Real value)
{
  outerMargin=value;
  if(geometry)
    geometry->margin = value;
}

void BLEG::ClearGeometry()
{
  SafeDelete(tempGeometry);
  if(geometryOwned)
    SafeDelete(geometry);
  geometryOwned = false;
}

void BLEG::SetTransform(const RigidTransform& T)
{
  geometry->SetTransform(T);
}

void BLEG::GetTransform(RigidTransform& T) const
{
  T = geometry->GetTransform();
}




//1 = pt, 2 = edge, 3 = face
inline int FeatureType(const Vector3& b) 
{
  int type=0;
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) type++;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) type++;
  return 3-type;
}

int EdgeIndex(const Vector3& b)
{
  if(FuzzyZero(b.x,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyZero(b.y,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyZero(b.z,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

int VertexIndex(const Vector3& b)
{
  if(FuzzyEquals(b.x,One,gBarycentricCoordZeroTolerance)) return 0;
  if(FuzzyEquals(b.y,One,gBarycentricCoordZeroTolerance)) return 1;
  if(FuzzyEquals(b.z,One,gBarycentricCoordZeroTolerance)) return 2;
  return 0;
  FatalError("Shouldn't get here");
  return -1;
}

Vector3 VertexNormal(const CollisionMesh& m,int tri,int vnum)
{
  Assert(!m.incidentTris.empty());
  int v=m.tris[tri][vnum];
  Vector3 n(Zero);
  for(size_t i=0;i<m.incidentTris[v].size();i++)
    n += m.TriangleNormal(m.incidentTris[v][i]);
  n.inplaceNormalize();
  return m.currentTransform.R*n;
}

Vector3 EdgeNormal(const CollisionMesh& m,int tri,int e)
{
  Assert(!m.triNeighbors.empty());
  Vector3 n=m.TriangleNormal(tri);
  if(m.triNeighbors[tri][e] != -1) {
    n += m.TriangleNormal(m.triNeighbors[tri][e]);
    n.inplaceNormalize();
  }
  return m.currentTransform.R*n;
}

///Compute normal from mesh geometry: returns the local normal needed for
///triangle 1 on m1 to get out of triangle 2 on m2.
///p1 and p2 are given in local coordinates
Vector3 ContactNormal(const CollisionMesh& m1,const CollisionMesh& m2,const Vector3& p1,const Vector3& p2,int t1,int t2)
{
  Triangle3D tri1,tri2;
  m1.GetTriangle(t1,tri1);
  m2.GetTriangle(t2,tri2);
  Vector3 b1=tri1.barycentricCoords(p1);
  Vector3 b2=tri2.barycentricCoords(p2);
  int type1=FeatureType(b1),type2=FeatureType(b2);
  switch(type1) {
  case 1:  //pt
    switch(type2) {
    case 1:  //pt
      //get the triangle normals
      {
	//printf("ODECustomMesh: Point-point contact\n");
	Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
	Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
	n2 -= n1;
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 2:  //edge
      {
	//printf("ODECustomMesh: Point-edge contact\n");
	Vector3 n1 = VertexNormal(m1,t1,VertexIndex(b1));
	int e = EdgeIndex(b2);
	Segment3D s = tri2.edge(e);
	Vector3 ev = m2.currentTransform.R*(s.b-s.a);
	Vector3 n2 = EdgeNormal(m2,t2,e);
	n2-=(n1-ev*ev.dot(n1)/ev.dot(ev)); //project onto normal
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 2:  //edge
    switch(type2) {
    case 1:  //pt
      {
	//printf("ODECustomMesh: Edge-point contact\n");
	Vector3 n2 = VertexNormal(m2,t2,VertexIndex(b2));
	int e = EdgeIndex(b1);
	Segment3D s = tri1.edge(e);
	Vector3 ev = m1.currentTransform.R*(s.b-s.a);
	Vector3 n1 = EdgeNormal(m1,t1,e);
	n2 = (n2-ev*ev.dot(n2)/ev.dot(ev))-n1; //project onto normal
	n2.inplaceNormalize();
	return n2;
      }
      break;
    case 2:  //edge
      {
	//printf("ODECustomMesh: Edge-edge contact\n");
	int e = EdgeIndex(b1);
	Segment3D s1 = tri1.edge(e);
	Vector3 ev1 = m1.currentTransform.R*(s1.b-s1.a);
	ev1.inplaceNormalize();
	e = EdgeIndex(b2);
	Segment3D s2 = tri2.edge(e);
	Vector3 ev2 = m2.currentTransform.R*(s2.b-s2.a);
	ev2.inplaceNormalize();
	Vector3 n; 
	n.setCross(ev1,ev2);
	Real len = n.length();
	if(len < gZeroNormalTolerance) {
	  //hmm... edges are parallel?
	}
	n /= len;
	//make sure the normal direction points into m1 and out of m2
	if(n.dot(m1.currentTransform*s1.a) < n.dot(m2.currentTransform*s2.a))
	  n.inplaceNegative();
	/*
	if(n.dot(m1.currentTransform.R*tri1.normal()) > 0.0) {
	  if(n.dot(m2.currentTransform.R*tri2.normal()) > 0.0) {
	    printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
	  }
	  n.inplaceNegative();
	}
	else {
	  if(n.dot(m2.currentTransform.R*tri2.normal()) < 0.0) {
	    printf("ODECustomMesh: Warning, inconsistent normal direction? %g, %g\n",n.dot(m1.currentTransform.R*tri1.normal()),n.dot(m2.currentTransform.R*tri2.normal()));
	  }
	}
	*/
	//cout<<"Edge vector 1 "<<ev1<<", vector 2" <<ev2<<", normal: "<<n<<endl;
	return n;
      }
      break;
    case 3:  //face
      return m2.currentTransform.R*tri2.normal();
    }
    break;
  case 3:  //face
    if(type2 == 3)
      printf("ODECustomMesh: Warning, face-face contact?\n");
    return m1.currentTransform.R*(-tri1.normal());
  }
  static int warnedCount = 0;
  if(warnedCount % 10000 == 0) 
    printf("ODECustomMesh: Warning, degenerate triangle, types %d %d\n",type1,type2);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

//Returns a contact normal for the closest point to the triangle t.  p is the point on the triangle.
//The direction is the one in which triangle 1 can move to get away from closestpt
Vector3 ContactNormal(const CollisionMesh& m,const Vector3& p,int t,const Vector3& closestPt)
{
  Triangle3D tri;
  m.GetTriangle(t,tri);
  Vector3 b=tri.barycentricCoords(p);
  int type=FeatureType(b);
  switch(type) {
  case 1:  //pt
    //get the triangle normal
    {
      Vector3 n = VertexNormal(m,t,VertexIndex(b));
      n.inplaceNegative();
      return n;
    }
    break;
  case 2:  //edge
    {
      int e = EdgeIndex(b);
      Vector3 n = EdgeNormal(m,t,e);
      n.inplaceNegative();
      return n;
    }
    break;
  case 3:  //face
    return m.currentTransform.R*(-tri.normal());
  }
  static int warnedCount = 0;
  if(warnedCount % 10000 == 0) 
    printf("ODECustomMesh: Warning, degenerate triangle, types %d\n",type);
  warnedCount++;
  //AssertNotReached();
  return Vector3(Zero);
}

int MeshMeshCollide(CollisionMesh& m1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,ContactPair* contact,int maxcontacts)
{
  CollisionMeshQuery q(m1,m2);
  bool res=q.WithinDistanceAll(outerMargin1+outerMargin2);
  if(!res) {
    return 0;
  }

  vector<int> t1,t2;
  vector<Vector3> cp1,cp2;
  q.TolerancePairs(t1,t2);
  q.TolerancePoints(cp1,cp2);
  //printf("%d Collision pairs\n",t1.size());
  const RigidTransform& T1 = m1.currentTransform;
  const RigidTransform& T2 = m2.currentTransform;
  RigidTransform T21; T21.mulInverseA(T1,T2);
  RigidTransform T12; T12.mulInverseA(T2,T1);
  Real tol = outerMargin1+outerMargin2;
  Real tol2 = Sqr(tol);

  size_t imax=t1.size();
  Triangle3D tri1,tri2,tri1loc,tri2loc;
  if(gDoTriangleTriangleCollisionDetection) {
    //test if more triangle vertices are closer than tolerance
    for(size_t i=0;i<imax;i++) {
      m1.GetTriangle(t1[i],tri1);
      m2.GetTriangle(t2[i],tri2);
      
      tri1loc.a = T12*tri1.a;
      tri1loc.b = T12*tri1.b;
      tri1loc.c = T12*tri1.c;
      tri2loc.a = T21*tri2.a;
      tri2loc.b = T21*tri2.b;
      tri2loc.c = T21*tri2.c;
      bool usecpa,usecpb,usecpc,usecpa2,usecpb2,usecpc2;
      Vector3 cpa = tri1.closestPoint(tri2loc.a);
      Vector3 cpb = tri1.closestPoint(tri2loc.b);
      Vector3 cpc = tri1.closestPoint(tri2loc.c);
      Vector3 cpa2 = tri2.closestPoint(tri1loc.a);
      Vector3 cpb2 = tri2.closestPoint(tri1loc.b);
      Vector3 cpc2 = tri2.closestPoint(tri1loc.c);
      usecpa = (cpa.distanceSquared(tri2loc.a) < tol2);
      usecpb = (cpb.distanceSquared(tri2loc.b) < tol2);
      usecpc = (cpc.distanceSquared(tri2loc.c) < tol2);
      usecpa2 = (cpa2.distanceSquared(tri1loc.a) < tol2);
      usecpb2 = (cpb2.distanceSquared(tri1loc.b) < tol2);
      usecpc2 = (cpc2.distanceSquared(tri1loc.c) < tol2);
      //if already existing, disable it
      if(usecpa && cpa.isEqual(cp1[i],cptol)) usecpa=false;
      if(usecpb && cpb.isEqual(cp1[i],cptol)) usecpb=false;
      if(usecpc && cpc.isEqual(cp1[i],cptol)) usecpc=false;
      if(usecpa2 && cpa2.isEqual(cp2[i],cptol)) usecpa2=false;
      if(usecpb2 && cpb2.isEqual(cp2[i],cptol)) usecpb2=false;
      if(usecpc2 && cpc2.isEqual(cp2[i],cptol)) usecpc2=false;
      
      if(usecpa) {
	if(usecpb && cpb.isEqual(cpa,cptol)) usecpb=false;
	if(usecpc && cpc.isEqual(cpa,cptol)) usecpc=false;
      }
      if(usecpb) {
	if(usecpc && cpc.isEqual(cpb,cptol)) usecpc=false;
      }
      if(usecpa2) {
	if(usecpb2 && cpb2.isEqual(cpa2,cptol)) usecpb2=false;
	if(usecpc2 && cpc2.isEqual(cpa2,cptol)) usecpc2=false;
      }
      if(usecpb) {
	if(usecpc2 && cpc.isEqual(cpb2,cptol)) usecpc2=false;
      }
      
      if(usecpa) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpa);
	cp2.push_back(tri2.a);
      }
      if(usecpb) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpb);
	cp2.push_back(tri2.b);
      }
      if(usecpc) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(cpc);
	cp2.push_back(tri2.c);
      }
      if(usecpa2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.a);
	cp2.push_back(cpa2);
      }
      if(usecpb2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.b);
	cp2.push_back(cpb2);
      }
      if(usecpc2) {
	t1.push_back(t1[i]);
	t2.push_back(t2[i]);
	cp1.push_back(tri1.c);
	cp2.push_back(cpc2);
      }
    }
    /*
    if(t1.size() != imax)
      printf("ODECustomMesh: Triangle vert checking added %d points\n",t1.size()-imax);
    */
    //getchar();
  }

  imax = t1.size();
  static int warnedCount = 0;
  for(size_t i=0;i<imax;i++) {
    m1.GetTriangle(t1[i],tri1);
    m2.GetTriangle(t2[i],tri2);

    tri1loc.a = T12*tri1.a;
    tri1loc.b = T12*tri1.b;
    tri1loc.c = T12*tri1.c;
    if(tri1loc.intersects(tri2)) { 
      if(warnedCount % 1000 == 0) {
	printf("ODECustomMesh: Triangles penetrate margin %g: can't trust contact detector\n",tol);
      }
      warnedCount++;
      /*
      //the two triangles intersect! can't trust results of PQP
      t1[i] = t1.back();
      t2[i] = t2.back();
      cp1[i] = cp1.back();
      cp2[i] = cp2.back();
      i--;
      imax--;
      */
    }
  }
  if(t1.size() != imax) {
    printf("ODECustomMesh: %d candidate points were removed due to collision\n",t1.size()-imax);
    t1.resize(imax);
    t2.resize(imax);
    cp1.resize(imax);
    cp2.resize(imax);
  }
  
  int k=0;  //count the # of contact points added
  for(size_t i=0;i<cp1.size();i++) {
    Vector3 p1 = T1*cp1[i];
    Vector3 p2 = T2*cp2[i];
    Vector3 n=p1-p2;
    Real d = n.norm();
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      n = ContactNormal(m1,m2,cp1[i],cp2[i],t1[i],t2[i]);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      continue;
    }
    else n /= d;
    //check for invalid normals
    Real len=n.length();
    if(len < gZeroNormalTolerance || !IsFinite(len)) continue;
    //cout<<"Local Points "<<cp1[i]<<", "<<cp2[i]<<endl;
    //cout<<"Points "<<p1<<", "<<p2<<endl;
    //Real utol = (tol)*0.5/d + 0.5;
    //contact[k].pos = p1+utol*(p2-p1);
    contact[k].pos = 0.5*(p1+p2) + ((outerMargin2 - outerMargin1)*0.5)*n;
    contact[k].normal = n;
    contact[k].depth = tol - d;
    if(contact[k].depth < 0) contact[k].depth = 0;
    contact[k].element1 = t1[i];
    contact[k].element2 = t2[i];
    //cout<<"Normal "<<n<<", depth "<<contact[i].depth<<endl;
    //getchar();
    k++;
    if(k == maxcontacts) break;
  }
  return k;
}

int MeshPointCloudCollide(CollisionMesh& m1,Real outerMargin1,CollisionPointCloud& pc2,Real outerMargin2,ContactPair* contact,int maxcontacts)
{
  Real tol = outerMargin1 + outerMargin2;
  int k=0;
  vector<int> tris;
  Triangle3D tri,triw;
  for(size_t i=0;i<pc2.points.size();i++) {
    Vector3 pw = pc2.currentTransform*pc2.points[i];
    NearbyTriangles(m1,pw,tol,tris,maxcontacts-k);
    for(size_t j=0;j<tris.size();j++) {   
      m1.GetTriangle(tris[j],tri);
      triw.a = m1.currentTransform*tri.a;
      triw.b = m1.currentTransform*tri.b;
      triw.c = m1.currentTransform*tri.c;
      Vector3 cp = triw.closestPoint(pw);
      Vector3 n = cp - pw;
      Real d = n.length();
      if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
	Vector3 plocal;
	m1.currentTransform.mulInverse(cp,plocal);
	n = ContactNormal(m1,plocal,tris[j],pw);
      }
      else if(d > tol) {  //some penetration -- we can't trust the result of PQP
	continue;
      }
      else n /= d;
      //migrate the contact point to the center of the overlap region
      contact[k].pos = 0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n;
      contact[k].normal = n;
      contact[k].depth = tol - d;
      contact[k].element1 = tris[j];
      contact[k].element2 = i;
      k++;
      if(k == maxcontacts) break;
    }
  }
  return k;
}

int PointCloudMeshCollide(CollisionPointCloud& pc1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,ContactPair* contact,int maxcontacts)
{
  int num = MeshPointCloudCollide(m2,outerMargin2,pc1,outerMargin1,contact,maxcontacts);
  for(int i=0;i<num;i++) ReverseContact(contact[i]);
  return num;
}

int MeshPrimitiveCollide(CollisionMesh& m1,Real outerMargin1,GeometricPrimitive3D& g2,const RigidTransform& T2,Real outerMargin2,ContactPair* contact,int maxcontacts)
{
  GeometricPrimitive3D gworld=g2;
  gworld.Transform(T2);
  Sphere3D s;
  if(gworld.type != GeometricPrimitive3D::Point && gworld.type != GeometricPrimitive3D::Sphere) {
    fprintf(stderr,"Distance computations between Triangles and %s not supported\n",gworld.TypeName());
    return 0;
  }
  if(gworld.type == GeometricPrimitive3D::Point) {
    s.center = *AnyCast<Point3D>(&gworld.data);
    s.radius = 0;
  }
  else {
    s = *AnyCast<Sphere3D>(&gworld.data);
  }
    
  Real tol = outerMargin1 + outerMargin2;
  Triangle3D tri;
  vector<int> tris;
  int k=0;
  NearbyTriangles(m1,gworld,tol,tris,maxcontacts);
  for(size_t j=0;j<tris.size();j++) {   
    m1.GetTriangle(tris[j],tri);
    tri.a = m1.currentTransform*tri.a;
    tri.b = m1.currentTransform*tri.b;
    tri.c = m1.currentTransform*tri.c;

    Vector3 cp = tri.closestPoint(s.center);
    Vector3 n = cp - s.center;
    Real nlen = n.length();
    Real d = nlen-s.radius;
    Vector3 pw = s.center;
    if(s.radius > 0)
      //adjust pw to the sphere surface
      pw += n*(s.radius/nlen);
    if(d < gNormalFromGeometryTolerance) {  //compute normal from the geometry
      Vector3 plocal;
      m1.currentTransform.mulInverse(cp,plocal);
      n = ContactNormal(m1,plocal,tris[j],pw);
    }
    else if(d > tol) {  //some penetration -- we can't trust the result of PQP
      continue;
    }
    else n /= nlen;
    //migrate the contact point to the center of the overlap region
    contact[k].pos = 0.5*(cp+pw) + ((outerMargin2 - outerMargin1)*0.5)*n;
    contact[k].normal = n;
    contact[k].depth = tol - d;
    contact[k].element1 = tris[j];
    contact[k].element2 = 0;
    k++;
    if(k == maxcontacts) break;
  }
  return k;
}

int PrimitiveMeshCollide(GeometricPrimitive3D& g1,const RigidTransform& T1,Real outerMargin1,CollisionMesh& m2,Real outerMargin2,ContactPair* contact,int maxcontacts)
{
  int num = MeshPrimitiveCollide(m2,outerMargin2,g1,T1,outerMargin1,contact,maxcontacts);
  for(int i=0;i<num;i++) ReverseContact(contact[i]);
  return num;
}

bool BLEG::Collides(const BLEG& o1, const BLEG& o2)
{
  ContactPair contact;
  return Contacts(o1,o2,&contact,1);
}

int BLEG::Contacts(const BLEG& o1, const BLEG& o2,ContactPair* contact,int m)
{
  int n=0;
  switch(o1.geometry->type) {
  case AnyGeometry3D::Primitive:
    switch(o2.geometry->type) {
    case AnyGeometry3D::Primitive:
      fprintf(stderr,"TODO: primitive-primitive collisions\n");
      break;
    case AnyGeometry3D::TriangleMesh:
      n = PrimitiveMeshCollide(o1.geometry->AsPrimitive(),o1.geometry->PrimitiveCollisionData(),o1.geometry->margin+o1.outerMargin,
			  o2.geometry->TriangleMeshCollisionData(),o2.geometry->margin+o2.outerMargin,
			  contact,m);
      break;
    case AnyGeometry3D::PointCloud:
      fprintf(stderr,"TODO: primitive-point cloud collisions\n");
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: primitive-implicit surface collisions\n");
      break;
    }
    break;
  case AnyGeometry3D::TriangleMesh:
    switch(o2.geometry->type) {
    case AnyGeometry3D::Primitive:
      n = MeshPrimitiveCollide(o1.geometry->TriangleMeshCollisionData(),o1.geometry->margin+o1.outerMargin,
			  o2.geometry->AsPrimitive(),o2.geometry->PrimitiveCollisionData(),o2.geometry->margin+o2.outerMargin,
			  contact,m);
      break;
    case AnyGeometry3D::TriangleMesh:
      n = MeshMeshCollide(o1.geometry->TriangleMeshCollisionData(),o1.geometry->margin+o1.outerMargin,
			  o2.geometry->TriangleMeshCollisionData(),o2.geometry->margin+o2.outerMargin,
			  contact,m);
      break;
    case AnyGeometry3D::PointCloud:
      n = MeshPointCloudCollide(o1.geometry->TriangleMeshCollisionData(),o1.geometry->margin+o1.outerMargin,
				o2.geometry->PointCloudCollisionData(),o2.geometry->margin+o2.outerMargin,
				contact,m);
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: triangle mesh-implicit surface collisions\n");
      break;
    }
    break;
  case AnyGeometry3D::PointCloud:
    switch(o2.geometry->type) {
    case AnyGeometry3D::Primitive:
      fprintf(stderr,"TODO: point cloud-primitive collisions\n");
      break;
    case AnyGeometry3D::TriangleMesh:
      n = PointCloudMeshCollide(*AnyCast<CollisionPointCloud>(&o1.geometry->collisionData),o1.geometry->margin+o1.outerMargin,
				   *AnyCast<CollisionMesh>(&o2.geometry->collisionData),o2.geometry->margin+o2.outerMargin,
				   contact,m);
      break;
    case AnyGeometry3D::PointCloud:
      fprintf(stderr,"TODO: point cloud-point cloud collisions\n");
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: point cloud-implicit surface collisions\n");
      break;
    }
    break;
  case AnyGeometry3D::ImplicitSurface:
    switch(o2.geometry->type) {
    case AnyGeometry3D::Primitive:
      fprintf(stderr,"TODO: implicit surface-primitive collisions\n");
      break;
    case AnyGeometry3D::TriangleMesh:
      fprintf(stderr,"TODO: implicit surface-triangle mesh collisions\n");
      break;
    case AnyGeometry3D::PointCloud:
      fprintf(stderr,"TODO: implicit surface-point cloud collisions\n");
      break;
    case AnyGeometry3D::ImplicitSurface:
      fprintf(stderr,"TODO: implicit surface-implicit surface collisions\n");
      break;
    }
    break;
  }
  for(int k=0;k<n;k++) {
    contact[k].id1 = o1.id;
    contact[k].id2 = o2.id;
  }
  return n;
}


void BLEG::GetAABB(Vector3& bmin,Vector3& bmax) const
{
  AABB3D bb = geometry->GetAABB();
  bb.bmin -= Vector3(outerMargin,outerMargin,outerMargin);
  bb.bmax += Vector3(outerMargin,outerMargin,outerMargin);
  bmin = bb.bmin;
  bmax = bb.bmax;
}

//given a list of items + counts, returns the item with the largest count.
//if multiple items have the same count, this just returns the first.
int Vote(const map<int,int>& counts)
{
  if(counts.empty()) return -1;
  int maxVal=counts.begin()->second,maxKey=counts.begin()->first;
  for(map<int,int>::const_iterator i=counts.begin();i!=counts.end();i++)
    if(i->second > maxVal) {
      maxVal = i->second;
      maxKey = i->first;
    }
  return maxKey;
}

void ClusterContactsMerge(vector<ContactPair>& contacts,int maxClusters,Real clusterNormalScale)
{
  if((int)contacts.size() <= maxClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].pos[0];
    pts[i][1] = contacts[i].pos[1];
    pts[i][2] = contacts[i].pos[2];
    pts[i][3] = contacts[i].normal[0]*clusterNormalScale;
    pts[i][4] = contacts[i].normal[1]*clusterNormalScale;
    pts[i][5] = contacts[i].normal[2]*clusterNormalScale;
    pts[i][6] = contacts[i].depth;
  }

  Timer timer;
  Statistics::HierarchicalClustering clust;
  clust.Build(pts,maxClusters,Statistics::HierarchicalClustering::AverageLinkage);
  //cout<<"Clustering time: "<<timer.ElapsedTime()<<endl;
  /*
  cout<<"Points"<<endl;
  for(size_t i=0;i<pts.size();i++)
    cout<<pts[i]<<endl;
  */
  
  //vote on the element
  vector<map<int,int> > element1voters(maxClusters),element2voters(maxClusters);
  for(int i=0;i<maxClusters;i++) {
    map<int,int>& e1 = element1voters[i];
    map<int,int>& e2 = element2voters[i];
    const vector<int>& inds = clust.Cluster(i);
    for(size_t j=0;j<inds.size();j++) {
      if(e1.count(contacts[inds[j]].element1)==0)
	e1[contacts[inds[j]].element1]=1;
      else
	e1[contacts[inds[j]].element1]++;
      if(e1.count(contacts[inds[j]].element2)==0)
	e2[contacts[inds[j]].element2]=1;
      else
	e2[contacts[inds[j]].element2]++;
    }
  }

  //read out the clusters
  contacts.resize(maxClusters);
  for(int i=0;i<maxClusters;i++) {
    const vector<int>& inds = clust.Cluster(i);
    /*
    cout<<"Cluster "<<i<<": ";
    for(size_t j=0;j<inds.size();j++)
      cout<<inds[j]<<", ";
    cout<<endl;
    */
    Vector mean(7,Zero);
    for(size_t j=0;j<inds.size();j++) 
      mean += pts[inds[j]];
    mean /= inds.size();

    contacts[i].pos[0] = mean[0];
    contacts[i].pos[1] = mean[1];
    contacts[i].pos[2] = mean[2];
    contacts[i].normal[0] = mean[3]/clusterNormalScale;
    contacts[i].normal[1] = mean[4]/clusterNormalScale;
    contacts[i].normal[2] = mean[5]/clusterNormalScale;
    contacts[i].element1 = Vote(element1voters[i]);
    contacts[i].element2 = Vote(element2voters[i]);

    Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ODESimulator: Warning, clustered normal became zero/infinite\n");
      int found = inds[0];
      contacts[i].pos[0] = pts[found][0];
      contacts[i].pos[1] = pts[found][1];
      contacts[i].pos[2] = pts[found][2];
      contacts[i].normal[0] = pts[found][3];
      contacts[i].normal[1] = pts[found][4];
      contacts[i].normal[2] = pts[found][5];
      Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
      contacts[i].normal[0] /= len;
      contacts[i].normal[1] /= len;
      contacts[i].normal[2] /= len;
      contacts[i].depth = pts[found][6];
      continue;
    }
    contacts[i].normal[0] /= len;
    contacts[i].normal[1] /= len;
    contacts[i].normal[2] /= len;
    contacts[i].depth = mean[6];
  }
}

void ClusterContactsKMeans(vector<ContactPair>& contacts,int maxClusters,Real clusterNormalScale)
{
  if((int)contacts.size() <= maxClusters) return;
  vector<Vector> pts(contacts.size());
  for(size_t i=0;i<pts.size();i++) {
    pts[i].resize(7);
    pts[i][0] = contacts[i].pos[0];
    pts[i][1] = contacts[i].pos[1];
    pts[i][2] = contacts[i].pos[2];
    pts[i][3] = contacts[i].normal[0]*clusterNormalScale;
    pts[i][4] = contacts[i].normal[1]*clusterNormalScale;
    pts[i][5] = contacts[i].normal[2]*clusterNormalScale;
    pts[i][6] = contacts[i].depth;
  }

  Statistics::KMeans kmeans(pts,maxClusters);
  //randomized
  if(gKmeansRandomInit)
    kmeans.RandomInitialCenters();
  else {
    //deterministic
    for(size_t i=0;i<kmeans.centers.size();i++)
      kmeans.centers[i] = kmeans.data[(i*pts.size())/kmeans.centers.size()];
  }
  int iters=gKmeansIters;
  kmeans.Iterate(iters);
  vector<map<int,int> > element1voters(kmeans.centers.size()),element2voters(kmeans.centers.size());
  for(size_t i=0;i<contacts.size();i++) {
    map<int,int>& e1 = element1voters[kmeans.labels[i]];
    map<int,int>& e2 = element2voters[kmeans.labels[i]];
    if(e1.count(contacts[i].element1)==0)
      e1[contacts[i].element1]=1;
    else
      e1[contacts[i].element1]++;
    if(e1.count(contacts[i].element2)==0)
      e2[contacts[i].element2]=1;
    else
      e2[contacts[i].element2]++;
  }
  contacts.resize(kmeans.centers.size());
  vector<int> degenerate;
  for(size_t i=0;i<contacts.size();i++) {
    //vote on the element
    contacts[i].pos[0] = kmeans.centers[i][0];
    contacts[i].pos[1] = kmeans.centers[i][1];
    contacts[i].pos[2] = kmeans.centers[i][2];
    contacts[i].normal[0] = kmeans.centers[i][3]/clusterNormalScale;
    contacts[i].normal[1] = kmeans.centers[i][4]/clusterNormalScale;
    contacts[i].normal[2] = kmeans.centers[i][5]/clusterNormalScale;
    contacts[i].element1 = Vote(element1voters[i]);
    contacts[i].element2 = Vote(element2voters[i]); 
    Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
    if(FuzzyZero(len) || !IsFinite(len)) {
      printf("ODESimulator: Warning, clustered normal became zero/infinite\n");
      //pick any in the cluster
      int found = -1;
      for(size_t k=0;k<kmeans.labels.size();k++) {
	if(kmeans.labels[k] == (int)i) {
	  found = (int)k;
	  break;
	}
      }
      if(found < 0) {
	//strange -- degenerate cluster?
	degenerate.push_back(i);
	continue;
      }
      contacts[i].pos[0] = pts[found][0];
      contacts[i].pos[1] = pts[found][1];
      contacts[i].pos[2] = pts[found][2];
      contacts[i].normal[0] = pts[found][3];
      contacts[i].normal[1] = pts[found][4];
      contacts[i].normal[2] = pts[found][5];
      Real len = Vector3(contacts[i].normal[0],contacts[i].normal[1],contacts[i].normal[2]).length();
      contacts[i].normal[0] /= len;
      contacts[i].normal[1] /= len;
      contacts[i].normal[2] /= len;
      contacts[i].depth = pts[found][6];
      continue;
    }
    contacts[i].normal[0] /= len;
    contacts[i].normal[1] /= len;
    contacts[i].normal[2] /= len;
    //cout<<"Clustered contact "<<contacts[i].pos[0]<<" "<<contacts[i].pos[1]<<" "<<contacts[i].pos[2]<<endl;
    //cout<<"Clustered normal "<<contacts[i].normal[0]<<" "<<contacts[i].normal[1]<<" "<<contacts[i].normal[2]<<endl;
    contacts[i].depth = kmeans.centers[i][6];
  }
  reverse(degenerate.begin(),degenerate.end());
  for(size_t i=0;i<degenerate.size();i++) {
    contacts.erase(contacts.begin()+degenerate[i]);
  }
}

void ClusterContacts(vector<ContactPair>& contacts,int maxClusters,Real clusterNormalScale)
{
  //for really big contact sets, do a subsampling
  if(contacts.size()*maxClusters > gMaxKMeansSize && contacts.size()*contacts.size() > gMaxHClusterSize) {
    int minsize = Max((int)gMaxKMeansSize/maxClusters,(int)Sqrt(Real(gMaxHClusterSize)));
    printf("ClusterContacts: subsampling %d to %d contacts\n",(int)contacts.size(),minsize);
    vector<dContactGeom> subcontacts(minsize);
    //random subsample
    if(gSubsampleRandomInit) {
      vector<int> subsample(contacts.size());
      RandomPermutation(subsample);
      subsample.resize(minsize);
      for(size_t i=0;i<subsample.size();i++)
	subcontacts[i] = contacts[subsample[i]];
    }
    else {
      //deterministic subsample
      for(int i=0;i<minsize;i++) {
	subcontacts[i] = contacts[(i*minsize)/contacts.size()];
      }
    }
    swap(subcontacts,contacts);
  }
  size_t hclusterSize = contacts.size()*contacts.size();
  size_t kmeansSize = contacts.size()*maxClusters;
  if(hclusterSize < gMaxHClusterSize)
    ClusterContactsMerge(contacts,maxClusters,clusterNormalScale);
  else 
    ClusterContactsKMeans(contacts,maxClusters,clusterNormalScale);
}


} //namespace KlamptCollide

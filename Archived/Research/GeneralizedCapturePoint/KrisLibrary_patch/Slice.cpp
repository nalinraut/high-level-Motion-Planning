#include "Slice.h"

namespace Geometry { 

void Slice(const Meshing::PointCloud3D& pc,const Plane3D& p,Real tol,vector<Vector3>& points)
{
  points.resize(0);
  for(size_t i=0;i<pc.points.size();i++)
    if(Abs(p.distance(pc.points[i])) <= tol) points.push_back(pc.points[i]);
}

void SliceXY(const Meshing::PointCloud3D& pc,const RigidTransform& T,Real tol,vector<Vector2>& points)
{
  Vector3 tp;
  Vector3 x,y,z;
  T.R.getRow1(x);
  T.R.getRow2(y);
  T.R.getRow3(z);
  points.resize(0);
  for(size_t i=0;i<pc.points.size();i++) {
    if(Abs(z.dot(pc.points[i])+T.t.z) <= tol) {
      points.push_back(Vector2(x.dot(pc.points[i])+T.t.x,y.dot(pc.points[i])+T.t.y));
    }
  }
}

///Returns the points in pc within tolerance tol of the plane p.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void Slice(const CollisionPointCloud& pc,const Plane3D& p,Real tol,vector<Vector3>& points);

///Same as slice, except slices the transformed point cloud T*pc about the
///x-y plane, and returns the points as x-y coordinates
void SliceXY(const CollisionPointCloud& pc,const RigidTransform& T,Real tol,vector<Vector2>& points);


///Returns the segments obtained by taking a slice of a mesh at plane p.
///O(T) time, where T is the number of triangles.
void Slice(const Meshing::TriMesh& m,const Plane3D& p,vector<Segment3D>& segments)
{
  segments.resize(0);
  Triangle3D t;
  Segment3D s;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle(i,t);
    if(t.intersects(p,s)) segments.push_back(s);
  } 
}

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const Meshing::TriMesh& m,const RigidTransform& T,vector<Segment2D>& segments)
{
  segments.resize(0);
  Vector3 x,y;
  Plane3D p;
  T.R.getRow1(x);
  T.R.getRow2(y);
  T.R.getRow3(p.normal);
  p.offset = T.t.z;
  Triangle3D t;
  Segment3D s;
  Segment2D s2;
  for(size_t i=0;i<m.tris.size();i++) {
    m.GetTriangle(i,t);
    if(t.intersects(p,s)) {
      s2.a.x = x.dot(s.a)+T.t.x;
      s2.a.y = y.dot(s.a)+T.t.y;
      s2.b.x = x.dot(s.b)+T.t.x;
      s2.b.y = y.dot(s.b)+T.t.y;
      segments.push_back(s2);
    }
  } 
}

///Returns the segments obtained by taking a slice of a mesh at plane p.
///Possibly faster than the naive Slice method due to the use of a bounding
///volume hierarchy.
void Slice(const CollisionMesh& m,const Plane3D& p,vector<Segment3D>& segments);

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const CollisionMesh& m,const RigidTransform& T,vector<Segment2D>& segments);

} //namespace Geometry

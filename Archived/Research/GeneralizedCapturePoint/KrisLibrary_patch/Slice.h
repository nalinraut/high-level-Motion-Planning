#ifndef GEOMETRY_SLICE_H
#define GEOMETRY_SLICE_H

#include <KrisLibrary/geometry/CollisionMesh.h>
#include <KrisLibrary/geometry/CollisionPointCloud.h>
#include <KrisLibrary/math3d/Segment2D.h>

namespace Geometry {

///Returns the points in pc within tolerance tol of the plane p.
///O(V) time, where V is the number of points.
void Slice(const Meshing::PointCloud3D& pc,const Plane3D& p,Real tol,vector<Vector3>& points);

///Same as slice, except slices the transformed point cloud T*pc about the
///x-y plane, and returns the points as x-y coordinates
void SliceXY(const Meshing::PointCloud3D& pc,const RigidTransform& T,Real tol,vector<Vector2>& points);

///Returns the points in pc within tolerance tol of the plane p.
///Possibly less than O(V) time due to the use of a bounding volume hierarchy.
void Slice(const CollisionPointCloud& pc,const Plane3D& p,Real tol,vector<Vector3>& points);

///Same as slice, except slices the transformed point cloud T*pc about the
///x-y plane, and returns the points as x-y coordinates
void SliceXY(const CollisionPointCloud& pc,const RigidTransform& T,Real tol,vector<Vector2>& points);


///Returns the segments obtained by taking a slice of a mesh at plane p.
///O(T) time, where T is the number of triangles.
void Slice(const Meshing::TriMesh& m,const Plane3D& p,vector<Segment3D>& segments);

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const Meshing::TriMesh& m,const RigidTransform& T,vector<Segment2D>& segments);

///Returns the segments obtained by taking a slice of a mesh at plane p.
///Possibly faster than the naive Slice method due to the use of a bounding
///volume hierarchy.
void Slice(const CollisionMesh& m,const Plane3D& p,vector<Segment3D>& segments);

///Same as Slice, except slices the transformed mesh T*m about the x-y plane,
///and returns the segments as x-y coordinates
void SliceXY(const CollisionMesh& m,const RigidTransform& T,vector<Segment2D>& segments);

} // namespace Geometry

#endif

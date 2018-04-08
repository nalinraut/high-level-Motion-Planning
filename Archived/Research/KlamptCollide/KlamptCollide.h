#ifndef KLAMPTCOLLIDE_H
#define KLAMPTCOLLIDE_H

#include <geometry/AnyGeometry.h>

namespace KlamptCollide {
  using namespace Math3D;
  using namespace Geometry;
  using namespace std;

struct ContactPair
{
  Vector3 pos;
  Vector3 normal;
  Real depth;
  int id1,id2;
  int element1,element2;
};

/** @brief A boundary-layer expanded geometry.
 * Usually a triangle mesh but could be a geometric primitive or point cloud.
 *
 * First: construct with a unique ID.  This will be maintained in the ContactPair data structure.
 * Second: load or set the geometry, and set up the desired margin.
 * Third: call SetTransform when the objects move.
 * Fourth: call BLEG::Collides and/or BLEG::Contacts to do either collision detection or contact
 *   generation.
 */
class BLEG
{
public:
  BLEG(int id=0,Real outerMargin=0.0);
  ~BLEG();
  void SetMargin(Real outerMargin);
  bool LoadGeometry(const char* fn);
  void SetGeometry(const GeometricPrimitive3D& geom);
  void SetGeometry(const Meshing::TriMesh& mesh);
  void SetGeometry(const vector<Vector3>& pointCloud);
  void SetGeometry(const Meshing::PointCloud3D& pointCloud);
  void SetGeometry(AnyCollisionGeometry3D* geometry);
  void ClearGeometry();
  void SetTransform(const RigidTransform& T);
  void GetTransform(RigidTransform& T) const;
  void GetAABB(Vector3& bmin,Vector3& bmax) const;

  ///returns true if the expanded geometry collides
  static bool Collides(const BLEG& g1,const BLEG& g2);
  ///computes the set of contact points (in world coordinates), returns
  ///the total number of contacts.
  static int Contacts(const BLEG& g1,const BLEG& g2,ContactPair* contacts,int maxContacts);

private:
  int id;  ///< the ID of this object, set on construction
  AnyGeometry3D* tempGeometry;
  AnyCollisionGeometry3D* geometry; 
  bool geometryOwned; ///<flag, whether the geometry pointer is owned by this object
  Real outerMargin; ///< don't set this directly
};

//postprocessing: cluster the contacts into some maximum number of clusters (maxClusters).
//the contacts array is used for both input of contacts and output of clustered contacts.
//clusterNormalScale gives how the normal component is weighted in the distance metric.
void ClusterContacts(vector<ContactPair>& contacts,int maxClusters,Real clusterNormalScale=0.1);

} //namespace KlamptCollide;

#endif

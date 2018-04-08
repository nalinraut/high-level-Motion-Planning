#include "KlamptCollide.h"
using namespace std;

int main(int argc,const char** argv)
{
  //load (or set) the geometry, set the outer margins
  printf("Loading meshes...\n");
  KlamptCollide::BLEG g1,g2;
  g1.LoadGeometry("test.DAE");
  g2.LoadGeometry("test.DAE");
  g1.SetMargin(0.01);
  g2.SetMargin(0.01);

  //set the transforms
  //shift geometry 2 over by a little
  Math3D::RigidTransformT1,T2;
  T1.setIdentity();
  T2.setIdentity();
  T2.t.set(0.42,0,0);
  g1.SetTransform(T1);
  g2.SetTransform(T2);

  printf("Collision checking...\n");
  //call contact generation
  vector<KlamptCollide::ContactPair> contacts(1000);
  int n=KlamptCollide::BLEG::Contacts(g1,g2,&contacts[0],1000);
  printf("%d contacts\n",n);
  contacts.resize(n);
  //if desired, run clustering
  if(n > 20) {
    printf("clustering to %d contacts\n",20);
    KlamptCollide::ClusterContacts(contacts,20);
  }
  return 1;
}

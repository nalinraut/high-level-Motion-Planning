#include <KrisLibrary/math/matrix.h>
#include <KrisLibrary/math/BLASInterface.h>
#include <KrisLibrary/math/LAPACKInterface.h>
#include <KrisLibrary/utils/StatCollector.h>

void TestBLASMV(int m,int n,int maxIters)
{
  Timer timer;
  StatCollector t,b;
  Matrix A(m,n);
  Vector v1,v2;
  for(int i=0;i<A.m;i++) {
    for(int j=0;j<A.n;j++) {
      A(i,j) = Rand();
    }
  }
  v1.resize(A.n);
  v2.resize(A.m);
  for(int i=0;i<v1.n;i++) v1(i) = Rand();
  for(int i=0;i<v2.n;i++) v2(i) = Rand();

  for(int iters=0;iters<maxIters;iters++) {
    timer.Reset();
    A.mul(v1,v2);
    t << timer.ElapsedTime();
  }

  std::swap(A.istride,A.jstride);
  for(int iters=0;iters<maxIters;iters++) {
    timer.Reset();
    BLASInterface::Mul(A,v1,v2);
    b << timer.ElapsedTime();
  }

  printf("Mat-vec mul (%d x %d): %gs (%g), BLAS %gs (%g)\n",A.m,A.n,t.average(),t.stddev(),b.average(),b.stddev());  
}

void TestBLASMM(int m,int n,int p,int maxIters)
{
  Timer timer;
  StatCollector t,b;
  Matrix A(m,n),B(n,p),C(m,p);
  for(int i=0;i<A.m;i++) {
    for(int j=0;j<A.n;j++) {
      A(i,j) = Rand();
    }
  }
  for(int i=0;i<B.m;i++) {
    for(int j=0;j<B.n;j++) {
      B(i,j) = Rand();
    }
  }

  for(int iters=0;iters<maxIters;iters++) {
    timer.Reset();
    C.mul(A,B);
    t << timer.ElapsedTime();
  }

  std::swap(A.istride,A.jstride);
  std::swap(B.istride,B.jstride);
  std::swap(C.istride,C.jstride);
  for(int iters=0;iters<maxIters;iters++) {
    timer.Reset();
    BLASInterface::Mul(A,B,C);
    b << timer.ElapsedTime();
  }

  printf("Mat mul (%d x %d x %d): %gs (%g), BLAS %gs (%g)\n",m,n,p,t.average(),t.stddev(),b.average(),b.stddev());  
}

void TestBLAS()
{
  BLASSelfTest();
  Vector v1(2000000),v2(2000000);
  for(int i=0;i<v1.n;i++) {
    v1(i) = Rand();
    v2(i) = Rand();
  }
  Timer timer;
  double t,b;

  timer.Reset();
  Real res=v1.dot(v2);
  t=timer.ElapsedTime();

  timer.Reset();
  Real res2=BLASInterface::Dot(v1,v2);
  b=timer.ElapsedTime();
  printf("Dot product (%d): %gs, BLAS %gs\n",v1.n,t,b);
  Assert(FuzzyEquals(res,res2));


  timer.Reset();
  res=v1.norm();
  t=timer.ElapsedTime();

  timer.Reset();
  res2=BLASInterface::Norm_L2(v1);
  b=timer.ElapsedTime();

  printf("Norm (%d): %gs, BLAS %gs\n",v1.n,t,b);
  Assert(FuzzyEquals(res,res2));

  /*
  TestBLASMV(100,100,1000);
  TestBLASMV(100,100000,100);
  TestBLASMV(1000,10000,100);
  TestBLASMV(100000,100,100);
  TestBLASMV(10000,1000,100);
  TestBLASMV(5000,5000,100);
  */
  TestBLASMM(100,100,100,100);
  TestBLASMM(400,400,400,10);
  TestBLASMM(100,1000,100,10);
}

void TestEigenvectors()
{
  Matrix temp(3,3);
  temp(0,0)=1; temp(0,1)=0.5; temp(0,2)=0.2;
  temp(1,0)=0.5; temp(1,1)=1; temp(1,2)=0.5;
  temp(2,0)=0.2; temp(2,1)=0.5; temp(2,2)=1;
  Vector lambda;
  Matrix Q;
  LAPACKInterface::Eigenvectors_Symmetric(temp,lambda,Q);
  cout<<"Eigenvalues: "<<VectorPrinter(lambda)<<endl;
  cout<<"Eigenvectors: "<<endl<<MatrixPrinter(Q)<<endl;
}


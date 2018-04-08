
#include "Polyhedron.h"
#include <iostream>
using namespace std;
using namespace Math;

Polyhedron::Polyhedron()
{
}

Polyhedron::Polyhedron(const Math::Matrix& M, const Math::Vector& V)
{
    initialize(M,V);
}

Polyhedron::~Polyhedron()
{
}

void Polyhedron::initialize(const Math::Matrix& M, const Math::Vector& V)
{
    A.copy(M);
    b.copy(V);
}

Polyhedron::Polyhedron(int _m, int _n, double* _A, double* _b)
{
    A.resize(_m,_n);
    A.copy(_A);
    b.resize(_m);
    b.copy(_b);
//    cout<<"A=\n"<<A<<endl;
//    cout<<"b=\n"<<b<<endl;
}

bool Polyhedron::contains(const Math::Vector& x)
{
    Math::Vector tmp;
    A.mul(x,tmp);
    assert(b.size() == tmp.size());
    for(int i= 0; i < b.size(); i++)
    {
        if(tmp(i) < b(i))
            return false;
    }
    return true;
}


bool Polyhedron::isEmpty()
{
    if(A.isEmpty() || b.empty())
        return true;
    return false;
}

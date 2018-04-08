/* 
 * File:   Polyhedron.h
 * Author: Jingru
 *
 * Created on November 28, 2011, 2:05 PM
 */

#ifndef POLYHEDRON_H
#define	POLYHEDRON_H

#include <math/matrix.h>
#include <math/vector.h>

class Polyhedron
{
public:
    Polyhedron();
    Polyhedron(const Math::Matrix& M, const Math::Vector& V);
    Polyhedron(int, int, double*, double*);
    ~Polyhedron();
    
    void initialize(const Math::Matrix &, const Math::Vector &);    
    bool contains(const Math::Vector& x);
    bool isEmpty();
        
    Math::Matrix A;
    Math::Vector b;
};

#endif	/* POLYHEDRON_H */


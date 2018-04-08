/* 
 * File:   PolyhedronStateChecker.h
 * Author: jingru
 *
 * Created on November 15, 2011, 4:12 PM
 */

#ifndef MYSTATEVALIDITYCHECKER_H
#define	MYSTATEVALIDITYCHECKER_H

#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <math/vector.h>
#include "Polyhedron.h"

#include "utility.h"

namespace ob = ompl::base;

class PolyhedronStateChecker : public ob::StateValidityChecker
{
public:
    PolyhedronStateChecker(const ob::SpaceInformationPtr &si):
    ob::StateValidityChecker(si){
        this->numD = si->getStateDimension();
        this->t_cc=0.0;
        this->reverseValidity = false;
    }
        
    void addPolyObt(const Polyhedron poly){
        polys.push_back(poly);
    }
    
    void addPolyObt(std::vector<Polyhedron> polylist){
        for(int i= 0; i < polylist.size(); i++){
            polys.push_back(polylist.at(i));
        }        
    }
    
    double getTCC(){
        return this->t_cc;
    }
    
    void setReverseValidity(){
    	this->reverseValidity = true;
    }

    void clearTimer(){
        this->t_cc = 0.0;
    }
    virtual bool isValid(const ob::State *state) const{
        const ob::RealVectorStateSpace::StateType *relstate = state->as<ob::RealVectorStateSpace::StateType>();
        Math::Vector v(numD,relstate->values);
        
      for(int i = 0; i < polys.size(); i++){
			Polyhedron poly = polys.at(i);
			if(poly.contains(v)){
				if(reverseValidity == true){
					return true;
				}
				return false;
			}
        }
		if(reverseValidity == true){
			return false;
		}
        return true;
    }   
    
    bool isValid(const Vector &v) const{
        for(int i = 0; i < polys.size(); i++){
                Polyhedron poly = polys.at(i);
                if(poly.contains(v)){
    				if(reverseValidity == true){
    					return true;
    				}
                   return false;
                }
        }
		if(reverseValidity == true){
			return false;
		}
       return true;
    }


    std::vector<Polyhedron> polys;
private:    
    int numD;
    //collision checking timer
    double t_cc;
    //if set to true, a point is valid if the polyhedrons contains it
    bool reverseValidity;
};

#endif	/* MYSTATEVALIDITYCHECKER_H */



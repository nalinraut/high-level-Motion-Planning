/* 
 * File:   ParameterizedTrajectory.h
 * Author: jingru
 *
 * Created on February 19, 2012, 10:27 AM
 */

#ifndef CEPATH_H
#define	CEPATH_H
#include "utility.h"
#include <vector>
#include <iostream>
#include <math/vector.h>
using namespace std;
typedef Math::Config Vector;

class CEPath {
public:
	CEPath() {
	}
	CEPath(int _nStates) {
		states.resize(_nStates);
	}
	CEPath(const CEPath& orig) {
		this->states = orig.states;
	}
	void print(std::ostream &out) {
		for (int i = 0; i < states.size(); i++) {
		  cout<<states[i]<<endl;
		}
	}

	~CEPath() {
	}
	void clear() {
		states.clear();
	}

	double length(Config &start, Config &goal) {
		double len = 0;
		Config *last = &start;
		int nStates = states.size();
		for (int i = 0; i < nStates; i++) {
		  len += last->distance(states[i]);
		  last = &states[i];
		}
		len += last->distance(goal);
		return len;
	}

	double pathlength() {
		double len = 0;
		CEState *last = &states[0];
		int nStates = states.size();
		for (int i = 1; i < nStates; i++) {
		  len += last->distance(states[i]);
		  last = &states[i];
		}
		return len;
	}

	void getVector(Math::Vector &v) {
		int nStates = states.size();
		if (states.empty()) {
			cout << "The trajectory is empty." << endl;
		}
		int numD = states[0].n;
		v.resize(nStates * numD);
		for (int i = 0; i < nStates; i++) {
			for (int j = 0; j < numD; j++) {
				v[i * numD + j] = states[i].q[j];
			}
		}
	}
	//get a trajectory from a vector as (state1, state2, ...)
	void setVector(Math::Vector &v, int _nStates) {
		int nStates = _nStates;
		if (!states.empty())
			states.clear();
		states.resize(nStates);
		int numD = v.size() / nStates;
		for (int i = 0; i < nStates; i++) {
			Config s(numD);
			for (int j = 0; j < numD; j++) {
				s.q[j] = v[i * numD + j];
			}
			states[i] = s;
		}
	}

	vector<Config> states;
};

#endif	/* PARATRAJECTORY_H */


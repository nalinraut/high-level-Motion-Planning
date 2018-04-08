/*
 * Linkage.h
 *
 *  Created on: Jan 19, 2014
 *      Author: iuiml
 */

#ifndef LINKAGE_H_
#define LINKAGE_H_

#include <vector>
#include <math3d/primitives.h>
#include <planning/CSpace.h>
#include <robotics/Frame.h>
#include <ostream>

using namespace std;
using namespace Math3D;

class Linkage {
public:
	Linkage( const Config& q, const Config& link_length);
	Linkage(int nD);
	virtual ~Linkage();
	void updateConfig( const Config& q);
	void updateCurrentConfig();
	void setLinkLength(const vector<double> &length);
	Vector2 getWorldPos( const int link, const Vector2& pos_local);
	bool selfCollision() const;
	void printJointPos(std::ostream &out);
private:
	Config q;
	int dim;
	Config link_length;
	vector<Frame2D> frames;
	vector<Frame2D> frames_world;
	vector<Vector2> pos_joints;

	void evalJointPos();
};

#endif /* LINKAGE_H_ */

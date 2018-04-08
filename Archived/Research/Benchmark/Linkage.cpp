/*
 * Linkage.cpp
 *
 *  Created on: Jan 19, 2014
 *      Author: iuiml
 */

#include "Linkage.h"
#include <assert.h>
#include <math3d/primitives.h>
#include <iostream>
#include "LineSegment.h"
#include "PlannerSettings.h"
using namespace std;
using namespace Math3D;

void Linkage::printJointPos(std::ostream &out){
	for(int i = 0; i < pos_joints.size(); i++){
		out << pos_joints[i]<<endl;
	}
}

Linkage::Linkage( const Config& q, const Config& link_length) {
	// TODO Auto-generated constructor stub
	assert( q.n == link_length.size());
	this->q = q;
	this->dim = this->q.n;

	this->link_length.resize( dim);
	for( int i = 0; i < dim; i++) {
		this->link_length[i] = link_length[i];
	}

	this->frames.resize( dim);
	this->frames_world.resize( dim);

	this->pos_joints.resize( dim + 1); //with base
	this->pos_joints[0].set( 0, 0);

	this->updateConfig( this->q);
	return;
}

Linkage::Linkage(int nD) {
	this->dim = nD;
	this->q.resize(dim);
	this->q.setZero();

	this->link_length.resize( dim);
	for( int i = 0; i < dim; i++) {
		this->link_length[i] = default_link_length;
	}

	this->frames.resize( dim);
	this->frames_world.resize( dim);

	this->pos_joints.resize( dim + 1); //with base
	this->pos_joints[0].set( 0, 0);

	this->updateConfig( this->q);
	return;
}

Linkage::~Linkage() {
	// TODO Auto-generated destructor stub
}

void Linkage::updateConfig(const Config& q) {
	this->frames[0].set( q[0], 0.0);
	for( int i = 1; i < this->dim; i++) {
		this->frames[i].set( q[i], Vector2( this->link_length[i-1], 0));
	}
//	for( int i = 0; i < this->dim; i++) {
//		cout << "Frame " << i << endl;
//		cout << frames[i] << endl;
//		cout << "**************************" << endl;
//	}

	this->frames_world[0].set( this->frames[0]);
	for( int i = 1; i < dim; i++) {
		this->frames_world[i].set( this->frames_world[i-1] * this->frames[i]);
	}
//	cout << "************************" << endl;
	this->evalJointPos();
//	cout << "************************" << endl;
}

Vector2 Linkage::getWorldPos(const int link, const Vector2& pos_local) {
	assert( link >= 0 && link < this->dim);

	Vector2 pos_world;
	this->frames_world[link].mulPoint( pos_local, pos_world);
	return pos_world;
}

bool Linkage::selfCollision() const {
	vector<LineSegment> lines;
	for( int i = 0; i < this->dim; i++) {
		lines.push_back( LineSegment( this->pos_joints[i], this->pos_joints[i+1]));
	}

	for( int i = 0; i < lines.size(); i++) {
		for( int j = i + 2; j < lines.size(); j++) {
			bool inCollision = LineSegment::intersect( lines[i], lines[j]);
			if( inCollision == true) {
				return true;
			}
		}
	}
	return false;
}

void Linkage::evalJointPos() {
	for( int i = 0; i < this->dim; i++) {
		this->pos_joints[i + 1] = this->frames_world[i] * Vector2( this->link_length[i], 0);
//		cout << this->pos_joints[i] << endl;
	}
}

/*
 * LineSegment.cpp
 *
 *  Created on: Jan 21, 2014
 *      Author: iuiml
 */

#include "LineSegment.h"

LineSegment::LineSegment( const Vector2& p1, const Vector2& p2) {
	// TODO Auto-generated constructor stub
	this->p1 = p1;
	this->p2 = p2;

	this->a = p1.y - p2.y;
	this->b = p2.x - p1.x;
	this->c = p1.x * p2.y - p2.x * p1.y;
}

LineSegment::~LineSegment() {
	// TODO Auto-generated destructor stub
}

bool LineSegment::intersect( LineSegment& l1, LineSegment& l2) {
	if( l1.onOneSide( l2))
		return false;

	if( l2.onOneSide( l1))
		return false;

	return true;
}

bool LineSegment::onOneSide(LineSegment& ls) {
	double res_p1 = ls.p1.x * a + ls.p1.y * b + c;
	double res_p2 = ls.p2.x * a + ls.p2.y * b + c;
	return res_p1 * res_p2 >= 0 ? true : false;
}

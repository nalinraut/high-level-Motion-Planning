/*
 * LineSegment.h
 *
 *  Created on: Jan 21, 2014
 *      Author: iuiml
 */

#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_

#include <math3d/primitives.h>
using namespace Math3D;

class LineSegment {
public:
	LineSegment( const Vector2& p1, const Vector2& p2);
	virtual ~LineSegment();
	static bool intersect( LineSegment& l1, LineSegment& l2);
private:
	//ax + by + c = 0;
	double a;
	double b;
	double c;
	Vector2 p1;
	Vector2 p2;
	bool onOneSide( LineSegment& ls);
};

#endif /* LINESEGMENT_H_ */

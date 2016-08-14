/*
 * spatial.cpp
 *
 *  Created on: 14.08.2016
 *      Author: JochenAlt
 */



#include "spatial.h"
#include "Util.h"

ostream& operator<<(ostream& os, const Point& p)
{
	cout << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}


Point::Point() {
	null();
}

Point::Point(const Point& p) {
	x = p.x;
	y = p.y;
	z = p.z;
}


void Point::null() {
	x = 0.0;
	y = 0.0;
	z = 0.0;
};

Point::Point(rational xP,rational yP, rational zP) {
	x = xP;
	y = yP;
	z = zP;
}

void Point::set(rational pX, rational pY,rational pZ) {
	x = pX;
	y = pY;
	z = pZ;
}


bool Point::isNull() {
	return 	(almostEqual(x,0.0,floatPrecision) &&
			almostEqual(y,0.0,floatPrecision) &&
			almostEqual(z,0.0,floatPrecision));
};

void Point::translate(const Point& pPoint) {
	x += pPoint.x;
	y += pPoint.y;
	z += pPoint.z;
}

void Point::mirrorAt(const Point& pMirror,rational scale) {
	x = pMirror.x + (pMirror.x-x)*scale;
	y = pMirror.y + (pMirror.y-y)*scale;
	z = pMirror.z + (pMirror.z-z)*scale;
}

void Point::mirrorAt(const Point& pMirror) {
	x = pMirror.x + (pMirror.x-x);
	y = pMirror.y + (pMirror.y-y);
	z = pMirror.z + (pMirror.z-z);
}

rational Point::distance(const Point& pPoint) const {
	return sqrt(sqr(pPoint.x-x) + sqr(pPoint.y-y)+  sqr(pPoint.z-z));
}

rational Point::length() const {
	return sqrt(sqr(x) + sqr(y)+  sqr(-z));
}

rational Point::angleToDegree(const Point& pPoint) const {
	return degrees(acos(scalarProduct(pPoint)/(length() * pPoint.length())));
}

rational Point::scalarProduct(const Point& pPoint) const {
	return x*pPoint.x + y*pPoint.y + z*pPoint.z;
}

Point Point::orthogonalProjection(const Point& pLine) const {
	// scalarProduct(a,b) / scalarProduct( line, line ) * line
	Point result = pLine;
	result *= scalarProduct(pLine)/ pLine.scalarProduct(pLine);
	return result;
}

Point Point::orthogonalProjection(const Point& pLineA, const Point& pLineB) const {
	Point translate(pLineA);
	Point selfTranslated=(*this);
	selfTranslated-= translate;
	Point line = pLineB;
	line -= translate;
	Point result = selfTranslated.orthogonalProjection(line);
	result += translate;
	return result;
}

Point Point::getPointOfLine(rational ratio, const Point& target) {
	ratio = constrain(ratio,0.0d,1.0d);
	Point result(*this);
	result.x += ratio*(float)(target.x - x);
	result.y += ratio*(float)(target.y - y);
	result.z += ratio*(float)(target.z - z);
	return result;
}


ostream& operator<<(ostream& os, const Rotation& p)
{
	cout << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}



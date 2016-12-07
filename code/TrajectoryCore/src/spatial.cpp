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

string Point::toString() const {
	stringstream str;
	str.precision(3);

	str << listStartToString("point",3)
		<< floatToString("x",x)
		<< floatToString("y",y)
		<< floatToString("z",z)
		<< listEndToString();

	return str.str();
}

bool Point::fromString(const string& str, int &idx) {
	int card;
	bool ok = listStartFromString("point", str, card, idx);
	ok = ok && floatFromString("x", str, x, idx);
	ok = ok && floatFromString("y", str, y, idx);
	ok = ok && floatFromString("z", str, z, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}

string Rotation::toString() const {
	stringstream str;
	str.precision(5);

	str << listStartToString("rot",3)
		<< floatToString("x",x)
		<< floatToString("y",y)
		<< floatToString("z",z)
		<< listEndToString();
	return str.str();
}

bool Rotation::fromString(const string& str, int &idx) {
	int card;
	bool ok = listStartFromString("rot", str, card, idx);
	ok = ok && floatFromString("x", str, x, idx);
	ok = ok && floatFromString("y", str, y, idx);
	ok = ok && floatFromString("z", str, z, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}


string Pose::toString() const {

	stringstream str;
	str.precision(3);
	str << listStartToString("pose",3);
	str << position.toString();
	str << orientation.toString();
	str << angles.toString();
	str << floatToString("gripper", gripperAngle);
	str << listEndToString();
	return str.str();
}

bool Pose::fromString(const string& str, int &idx) {
	int card;
	bool ok = listStartFromString("pose", str, card, idx);
	ok = ok && position.fromString(str, idx);
	ok = ok && orientation.fromString(str,idx);
	ok = ok && angles.fromString(str,idx);
	ok = ok && floatFromString("gripper", str, gripperAngle, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}


ostream& operator<<(ostream& os, const JointAngles& p)
{
	cout << std::setprecision(2) << "(";
	for (int i = 0;i<NumberOfActuators;i++) {
		if (i>0)
			cout << ",";
		cout << std::fixed << p.a[i];
	}
	cout << ")";
	return os;
}


bool TrajectoryNode::fromString(const string& str, int &idx) {

	int card;
	bool ok = listStartFromString("tnode", str,card,idx);
    pose.fromString(str,idx);
    ok = ok && floatFromString("duration", str, duration, idx);
    ok = ok && floatFromString("distance", str, startSpeed, idx);
    ok = ok && floatFromString("averagespeed", str, averageSpeedDef, idx);
    ok = ok && floatFromString("startSpeed", str, startSpeed, idx);

    ok = ok && stringFromString("name", str, name, idx);
    int interpolationTypeInt;
    ok = ok && intFromString("type", str, interpolationTypeInt, idx);
    interpolationTypeDef = (InterpolationType)interpolationTypeInt;
    int x;
    ok = ok && intFromString("time", str, x, idx);
    time = x;
    ok = ok && listEndFromString(str,idx);
    return ok;
}


string TrajectoryNode::toString() const {
	stringstream str;
	str.precision(3);
	str << listStartToString("tnode",5);
	str << pose.toString();
	str << floatToString("duration", duration);
	str << floatToString("distance", distance);
	str << floatToString("averagespeed", averageSpeedDef);
	str << floatToString("startSpeed", startSpeed);
	str << stringToString("name", name);
	str << intToString("type", (int)interpolationTypeDef);
	str << intToString("time",time);
	str << listEndToString();

	return str.str();
}


string JointAngles::toString() const {
	stringstream str;
	str.precision(3);
	str << listStartToString("angles", 7);
	for (int i = 0;i<7;i++)
		str << floatToString(int_to_string(i),a[i]);
	str << listEndToString();
	return str.str();
}

bool JointAngles::fromString(const string& str, int& idx){
	int card;
	bool ok = listStartFromString("angles", str, card, idx);
    for (int i = 0;i<7;i++) {
    	ok = ok && floatFromString(int_to_string(i), str, a[i], idx);
    }

	ok = ok && listEndFromString(str, idx);

    return ok;
}

ostream& operator<<(ostream& os, const Rotation& p)
{
	cout << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}

string TrajectoryNode::getText() {

		int par[7];
		int j=0;
		for (int i = 0;i<3;i++)
			par[j++] = pose.position[i];

		for (int i = 0;i<3;i++)
			par[j++] = pose.orientation[i];

		par[j++] = pose.gripperAngle;

		string text = string_format("%s (%i,%i,%i)(%i,%i,%i)(%i)",
							name.c_str(),
							par[0],par[1],par[2],
							par[3],par[4],par[5],
							par[6]);
		return text;
	}



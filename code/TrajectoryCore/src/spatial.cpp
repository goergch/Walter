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
	str << "{point{x=" << x << ";y=" << y << ";z=" << z << ")}";

	return str.str();
}

bool Point::fromString(const string& str, int &idx) {
    int noOfItems = sscanf(&(str.c_str()[idx]),"{point{%lf;%lf;%lf)}%n", &x, &y, &z, &idx);
    return (noOfItems == 4);
}

string Rotation::toString() const {
	stringstream str;
	str.precision(3);
	str << "{rot{x=" << x << ";y=" << y << ";z=" << z << ")}";

	return str.str();
}

bool Rotation::fromString(const string& str, int &idx) {
    int noOfItems = sscanf(&(str.c_str()[idx]),"{rot{%lf;%lf;%lf)}%n", &x, &y, &z, &idx);
    return (noOfItems == 4);
}


string Pose::toString() const {

	stringstream str;
	str.precision(3);
	str << "{pose{" << position.toString() << orientation.toString() << floatToString("gripper",gripperAngle) << "};";

	return str.str();
}

bool Pose::fromString(const string& str, int &idx) {
	int noOfItems = sscanf(&(str.c_str()[idx]),"{pose{%n", &idx);
	bool ok = position.fromString(string(&(str.c_str()[idx])),idx);
	ok = ok && orientation.fromString(string(&(str.c_str()[idx])),idx);
	ok = ok && floatFromString(string("gripper"),string(&(str.c_str()[idx])), gripperAngle, idx);
	noOfItems += sscanf(&(str.c_str()[idx]),"}%n", &idx);

	return ok && (noOfItems == 2);
}


bool TrajectoryNode::fromString(const string& str, int &idx) {
    int noOfItems = sscanf(&(str.c_str()[idx]),"{tnode{%n", &idx);
    pose.fromString(str,idx);
    bool ok = jointAnglesFromString("angles", str, angles, idx);
    if (!ok)
    	LOG(ERROR) << "parse error joint angles";
    ok = intFromString("duration", str, duration_ms, idx);
    if (!ok)
    	LOG(ERROR) << "parse error duration";

    ok = stringFromString("name", str, name, idx);
    if (!ok)
    	LOG(ERROR) << "parse error name";

    int interpolationTypeInt;
    ok = intFromString("type", str, interpolationTypeInt, idx);
    if (!ok)
    	LOG(ERROR) << "parse error interpolation type";

    interpolationType = (InterpolationType)interpolationTypeInt;
    ok = intFromString("time", str, time_ms, idx);
    if (!ok)
    	LOG(ERROR) << "parse error interpolation time";

    noOfItems += sscanf(&(str.c_str()[idx]),"}%n", &idx);

    return ok && (noOfItems == 2);
}


string TrajectoryNode::toString() const {
	stringstream str;
	str.precision(3);
	str << "{tnode{" << pose.toString() << jointAnglesToString("angles", angles)
		<< intToString("duration", duration_ms)
		<< stringToString("name", name) << intToString("type", (int)interpolationType)
		<< intToString("time",time_ms)
		<< "}";

	return str.str();
}

string floatToString(const string& tag, double x) {
	stringstream str;
	str.precision(3);
	str << "{" << tag << "=" << x << "}";
	return str.str();
}

bool floatFromString (const string& tag, const string& str, double &x, int& idx) {
	string parseStr = "{" + tag + "=%lf}%n";
    int noOfItems = sscanf(&(str.c_str()[idx]),parseStr.c_str(), &x, &idx);
    return (noOfItems == 2);
}

string intToString(const string& tag, int x) {
	stringstream str;
	str.precision(3);
	str << "{" << tag << "=" << x << "}";
	return str.str();
}

bool intFromString (const string& tag, const string& str, int &x, int& idx) {
	string parseStr = "{" + tag + "=%i}%n";
    int noOfItems = sscanf(&(str.c_str()[idx]),parseStr.c_str(), &x, &idx);
    return (noOfItems == 2);
}
string stringToString(const string& tag, const string& x) {
	stringstream str;
	str.precision(3);
	str << "{" << tag << "=" << x << "}";
	return str.str();
}

bool stringFromString (const string& tag, const string& str, string &x, int& idx) {
	string parseStr = "{" + tag + "=%s}%n";
    int noOfItems = sscanf(&(str.c_str()[idx]),parseStr.c_str(), &x, &idx);
    return (noOfItems == 2);
}

string jointAnglesToString(const string& tag, const JointAngleType& x) {
	stringstream str;
	str.precision(3);
	str << "{" << tag << "={";
	for (int i = 0;i<7;i++)
		str << floatToString(int_to_string(i),x[i]);
	str << "}";
	return str.str();
}

bool jointAnglesFromString (const string& tag, const string& str, JointAngleType &x, int& idx) {
	string parseStr = "{" + tag + "={%n";
    int noOfItems = sscanf(&(str.c_str()[idx]),parseStr.c_str(), &x, &idx);
    bool sumOk = true;;
    for (int i = 0;i<7;i++) {
    	rational x;
    	bool ok = floatFromString(int_to_string(i), str, x, idx);
    	sumOk = sumOk & ok;
    	if (!ok)
    		LOG(ERROR) << "error parsing joint angles";
    }

    noOfItems += sscanf(&(str.c_str()[idx]),"}%n", &idx);
    return sumOk && (noOfItems == 2);
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



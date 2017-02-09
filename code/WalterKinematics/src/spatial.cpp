#include "spatial.h"
#include "Util.h"

ostream& operator<<(ostream& os, const Point& p)
{
	os << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
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

string Point::toString(int & indent) const {
	stringstream str;
	str.precision(3);

	str << listStartToString("point",indent)
		<< floatToString("x",x)
		<< floatToString("y",y)
		<< floatToString("z",z)
		<< listEndToString(indent);

	return str.str();
}

string Point::toString(string tag, int & indent) const {
	stringstream str;
	str.precision(3);

	str << listStartToString(tag,indent)
		<< floatToString("x",x)
		<< floatToString("y",y)
		<< floatToString("z",z)
		<< listEndToString(indent);

	return str.str();
}


bool Point::fromString(const string& str, int &idx) {
	bool ok = listStartFromString("point", str, idx);
	ok = ok && floatFromString("x", str, x, idx);
	ok = ok && floatFromString("y", str, y, idx);
	ok = ok && floatFromString("z", str, z, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}

bool Point::fromString(string tag, const string& str, int &idx) {
	bool ok = listStartFromString(tag, str, idx);
	ok = ok && floatFromString("x", str, x, idx);
	ok = ok && floatFromString("y", str, y, idx);
	ok = ok && floatFromString("z", str, z, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}

string Rotation::toString(int & indent) const {
	return Point::toString("rot", indent);
}

bool Rotation::fromString(const string& str, int &idx) {
	return Point::fromString("rot", str, idx);
}

ostream& operator<<(ostream& os, const Pose& p)
{
	os << std::setprecision(2) << "( angles=" << p.angles << ", pos=" << p.position << ",ori=" << p.orientation << ", tcp=" << p.tcpDeviation << ")";
	return os;
}

string Pose::toString(int & indent) const {

	stringstream str;
	str.precision(3);
	str << listStartToString("pose", indent);
	str << endofline(indent);
	str	<< position.toString(indent);
	str << endofline(indent);
	str << orientation.toString(indent);
	str << endofline(indent);
	str << angles.toString(indent);
	str << endofline(indent);
	str << tcpDeviation.toString("tcp",indent);
	str << endofline(indent);
	str << floatToString("gripper", gripperAngle);
	str << endofline(indent-1);
	str << listEndToString(indent);
	return str.str();
}

bool Pose::fromString(const string& str, int &idx) {
	bool ok = listStartFromString("pose", str, idx);
	ok = ok && position.fromString(str, idx);
	ok = ok && orientation.fromString(str,idx);
	ok = ok && angles.fromString(str,idx);
	ok = ok && tcpDeviation.fromString("tcp", str,idx);
	ok = ok && floatFromString("gripper", str, gripperAngle, idx);
	ok = ok && listEndFromString(str,idx);
	return ok;
}


ostream& operator<<(ostream& os, const JointAngles& p)
{
	os << std::setprecision(2) << "(";
	for (int i = 0;i<NumberOfActuators;i++) {
		if (i>0)
			os << ",";
		os << std::fixed << p.a[i];
	}
	os << ")";
	return os;
}


ostream& operator<<(ostream& os, const TrajectoryNode& n)
{
	os << std::setprecision(1) << "( pose=" << n.pose << "}" ;
    return os;
}

bool TrajectoryNode::fromString(const string& str, int &idx) {

	bool ok = listStartFromString("tnode", str,idx);
    pose.fromString(str,idx);
    ok = ok && intFromString("durationdef", str, durationDef, idx);
    ok = ok && floatFromString("averagespeeddef", str, averageSpeedDef, idx);
    ok = ok && boolFromString("continouslydef", str, continouslyDef, idx);

    ok = ok && floatFromString("duration", str, duration, idx);
    ok = ok && floatFromString("distance", str, distance, idx);
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


string TrajectoryNode::toString(int & indent) const {
	stringstream str;
	str.precision(3);
	str << listStartToString("tnode",indent);
	str << endofline(indent);
	str << pose.toString(indent);
	str << endofline(indent);
	str << intToString("durationdef", durationDef);
	str << floatToString("averagespeeddef", averageSpeedDef);
	str << boolToString("continouslydef", continouslyDef);

	str << floatToString("duration", duration);
	str << floatToString("distance", distance);
	str << floatToString("startSpeed", startSpeed);
	str << stringToString("name", name);
	str << intToString("type", (int)interpolationTypeDef);
	str << intToString("time",time);
	str << listEndToString(indent);
	str << endofline(indent);

	return str.str();
}


string JointAngles::toString(int& indent) const {
	stringstream str;
	str.precision(3);
	str << listStartToString("angles", indent);
	for (int i = 0;i<7;i++)
		str << floatToString(int_to_string(i),a[i]);
	str << listEndToString(indent);
	return str.str();
}

bool JointAngles::fromString(const string& str, int& idx){
	bool ok = listStartFromString("angles", str, idx);

	for (int i = 0;i<7;i++) {
    	ok = ok && floatFromString(int_to_string(i), str, a[i], idx);
    }
	ok = ok && listEndFromString(str, idx);

    return ok;
}

ostream& operator<<(ostream& os, const Rotation& p)
{
	os << std::setprecision(1) << "(" << p.x << "," << p.y << "," << p.z << ")";
    return os;
}

string TrajectoryNode::getText() const {

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



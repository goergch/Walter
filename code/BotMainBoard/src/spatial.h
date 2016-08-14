/*
 * spatial.h
 *
 *  Created on: 07.08.2016
 *      Author: JochenAlt
 */

#ifndef SPATIAL_H_
#define SPATIAL_H_

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#include "cmatrix"
#pragma GCC diagnostic pop

#include "Setup.h"
#include "Util.h"
using techsoft::mslice;
typedef techsoft::matrix<rational>  Matrix;
typedef techsoft::matrix<rational>  HomMatrix;
typedef std::valarray<rational> HomVector;
typedef std::valarray<rational> Vector;
typedef std::valarray<rational> JointAngleType;

enum InterpolationType { LINEAR, CUBIC_BEZIER };


class Point {
	 friend ostream& operator<<(ostream&, const Point&);
	public:
		Point();
		Point(const Point& p);
		Point(const HomVector& p) {
			x = p[X];
			y = p[Y];
			z = p[Z];
		}

		Point(rational xP,rational yP, rational zP);
		void translate(const Point& pPoint);
		void mirrorAt(const Point& pPoint, rational scale);
		void mirrorAt(const Point& pPoint);
		void set(rational pX, rational pY,rational pZ);
		void null();
		bool isNull();


		void operator= (const Point& p) {
			x = p.x;
			y = p.y;
			z = p.z;
		}

		void operator= (const HomVector& p) {
			x = p[X];
			y = p[Y];
			z = p[Z];
		}

		void operator+= (const Point& p) {
			x += p.x;
			y += p.y;
			z += p.z;
		}

		void operator-= (const Point& p) {
			x -= p.x;
			y -= p.y;
			z -= p.z;
		}

		void operator*= (const rational f) {
			x *= f;
			y *= f;
			z *= f;
		}

		void operator/= (const rational f) {
			float xrf= 1.0/f;
			x *= xrf;
			y *= xrf;
			z *= xrf;
		}

		Point operator- (const Point& p) const{
			Point result= (*this);
			result -= p;
			return result;
		}

		Point operator+ (const Point& p) const{
			Point result= (*this);
			result += p;
			return result;
		}

		Point operator/ (const rational f) const{
			Point result= (*this);
			result*= (1./f);
			return result;
		}

		Point operator* (const rational f) const{
			Point result= (*this);
			result*=f;
			return result;
		}


		bool operator==(const Point& pos) {
			return almostEqual(x,pos.x, floatPrecision) && almostEqual(y, pos.y, floatPrecision) && almostEqual(z, pos.z, floatPrecision);
		};

		bool operator!=(const Point& pos) {
			return !((*this) == pos);
		};


		rational& operator[] (int idx)  {
			switch (idx) {
				case X:	return x;break;
				case Y:	return y;break;
				case Z:	return z;break;
				default:
				break;
			}
			LOG(ERROR) << "Point[" << idx << "] out of bounds";
			return x;
		};

		rational operator[] (int idx)  const {
			switch (idx) {
				case X:	return x;break;
				case Y:	return y;break;
				case Z:	return z;break;
				default:
				break;
			}
			LOG(ERROR) << "Point[" << idx << "] out of bounds";
			return x;
		};

		rational distance(const Point& p) const;
		rational length() const;
		rational angleToDegree(const Point& pPoint) const;
		rational scalarProduct(const Point& pPoint) const;
		Point orthogonalProjection(const Point& pLine) const;
		Point orthogonalProjection(const Point& pLineA, const Point &pLineB) const;
		Point getPointOfLine(rational ratio, const Point& target);

		HomVector getHomVector() const {
			HomVector result = { x,y,z,1.0 };
			return result;
		}


		rational x;
		rational y;
		rational z;


};


class Rotation : public Point {
	 friend ostream& operator<<(ostream&, const Rotation&);
	public:
		Rotation () : Point (0,0,0){};
		Rotation(float xP,float yP, float zP): Point(xP,yP,zP) {
			x = xP;
			y = yP;
			z = zP;
		}

		Rotation(const Rotation& p) : Point(p) {
			x= p.x;
			y= p.y;
			z= p.z;
		};
		void operator=(const Rotation& rot) {
			x= rot.x;
			y= rot.y;
			z= rot.z;
		};
		void operator+=(const Rotation& rot) {
			x += rot.x;
			y += rot.y;
			z += rot.z;
		};

		void operator*=(float f) {
			x *= f;
			y *= f;
			z *= f;
		};
		void operator/=(float f) {
			x /= f;
			y /= f;
			z /= f;
		};
		Rotation operator*(const float f) const {
			Rotation result(*this);
			result.x *= f;
			result.y *= f;
			result.z *= f;
			return result;
		};
		Rotation operator/(const float f) const {
			Rotation result(*this);
			result *= (1./f);
			return result;
		};

		Rotation operator+(const Rotation& rot) const {
			Rotation result(*this);
			result += rot;
			return result;
		};

		Rotation operator-(const Rotation& rot) const {
			Rotation result(*this);
			result -= rot;
			return result;
		};
		bool operator==(const Rotation& pos) {
			return (x == pos.x) && (y == pos.y) && (z == pos.z);
		};

		bool operator!=(const Rotation& pos) {
			return !((*this) == pos);
		};

};


class Pose {
	public:
		Pose() {
			null();
		};
		Pose(const Pose& pose): Pose() {
			position = pose.position;
			orientation = pose.orientation;
			gripperAngle = pose.gripperAngle;
		};
		Pose(const Point& pPosition, const Rotation& pOrientation, const rational pGripperAngle) {
			position = pPosition;
			orientation = pOrientation;
			gripperAngle = pGripperAngle;
		};

		void operator= (const Pose& pose) {
			position = pose.position;
			orientation = pose.orientation;
			gripperAngle = pose.gripperAngle;
		}
		void null() {
			orientation.null();
			position.null();
			gripperAngle = 0.0;
		}
		bool isNull() {
			return position.isNull();
		}

		void mirrorAt(const Pose& pMirror) {
			position.mirrorAt(pMirror.position);
			orientation.mirrorAt(pMirror.orientation);
		}

		float distance(const Pose& pPose) const {
			return sqrt((pPose.position[0]-position[0])*(pPose.position[0]-position[0]) +
						(pPose.position[1]-position[1])*(pPose.position[1]-position[1]) +
						(pPose.position[2]-position[2])*(pPose.position[2]-position[2]));
		}

		float length() const{
			return sqrt(sqr(position[0]) + sqr(position[1]) + sqr(position[2]));
		}

		bool operator==(const Pose& pPose) {
			return 	(position == pPose.position &&
					orientation == pPose.orientation);
		};

		bool operator!=(const Pose& pos) {
			return !((*this) == pos);
		};

		void operator+=(const Pose& pos) {
			position += pos.position;
			for (int i = 0;i<3;i++)
				orientation[i] += pos.orientation[i];
		};
		void operator-=(const Pose& pos) {
			position -= pos.position;
			for (int i = 0;i<3;i++)
				orientation[i] -= pos.orientation[i];
		};

		void operator*=(const float x) {
			position *= x;

			for (int i = 0;i<3;i++)
				orientation[i] *= x;
		};
		void operator/=(const float x) {
			position /= x;
			for (int i = 0;i<3;i++)
				orientation[i] /= x;
		};

		Pose operator*(float x) const {
			Pose result(*this);
			result *= x;
			return result;
		};

		Pose operator/(float x) const {
			Pose result(*this);
			result /= x;
			return result;
		};
		Pose  operator+(const Pose& pos) const {
			Pose result(*this);
			result += pos;
			return result;
		};

		Pose operator-(const Pose& pos) const {
			Pose result(*this);
			result -= pos;
			return result;
		};



	Point position;
	Rotation orientation;
	rational gripperAngle;
};


class TrajectoryNode {
public:

	TrajectoryNode() {
		smooth = 0;
		duration_ms = 0;
		angles = { 0,0,0,0,0,0,0 };
		interpolationType = CUBIC_BEZIER;
		time_ms = 0;
		pose.null();
	}
	TrajectoryNode(const TrajectoryNode& par) {
		smooth = par.smooth;
		duration_ms = par.duration_ms;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
		interpolationType = par.interpolationType;
		time_ms = par.time_ms;
	}
	void operator= (const TrajectoryNode& par) {
		smooth = par.smooth;
		duration_ms = par.duration_ms;
		name = par.name;
		pose = par.pose;
		angles = par.angles;
		interpolationType = par.interpolationType;
		time_ms = par.time_ms;
	}


	string getText() {

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
	bool isNull() {
		return pose.isNull();
	}
	void null() {
		pose.null();
	}
	Pose pose;
	JointAngleType angles;
	bool smooth;
	int duration_ms;
	string name;
	InterpolationType interpolationType;
	int time_ms;
};



#endif /* SPATIAL_H_ */

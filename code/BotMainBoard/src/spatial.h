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
		Pose(const HomVector& pPosition, const HomVector& pOrientation, const rational pGripperAngle) {
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
			orientation = { 0.0, 0.0, 0.0, 1.0};
			position 	= { 0.0, 0.0, 0.0, 1.0};
			gripperAngle = 0.0;
		}
		bool isNull() {
			return (almostEqual(position[0],0.0,floatPrecision) &&
					almostEqual(position[1],0.0,floatPrecision) &&
					almostEqual(position[2],0.0,floatPrecision) &&
					almostEqual(position[3],1.0,floatPrecision));
		}

		void mirrorAt(const Pose& pMirror) {
			for (int i = 0;i<3;i++)
				position[i] = position[i] + (pMirror.position[i]-position[i]);
			for (int i = 0;i<3;i++)
				orientation[i] = orientation[i] + (pMirror.orientation[i]-orientation[i]);
		}

		float distance(const Pose& pPose) const {
			return sqrt(sqr(pPose.position[0]-position[0]) + sqr(pPose.position[1]-position[1]) + sqr(pPose.position[2]-position[2]));
		}

		float length() const{
			return sqrt(sqr(position[0]) + sqr(position[1]) + sqr(position[2]));
		}

		bool operator==(const Pose& pPose) {
			return 	(almostEqual(pPose.position[0]	,position[0],	floatPrecision) &&
					almostEqual(pPose.position[1]	,position[1],	floatPrecision) &&
					almostEqual(pPose.position[2]	,position[2],	floatPrecision) &&
					almostEqual(pPose.orientation[0],orientation[0],floatPrecision) &&
					almostEqual(pPose.orientation[1],orientation[1],floatPrecision) &&
					almostEqual(pPose.orientation[2],orientation[2],floatPrecision));
		};

		bool operator!=(const Pose& pos) {
			return !((*this) == pos);
		};

		void operator+=(const Pose& pos) {
			for (int i = 0;i<3;i++)
				position[i] += pos.position[i];
			for (int i = 0;i<3;i++)
				orientation[i] += pos.orientation[i];
		};
		void operator-=(const Pose& pos) {
			for (int i = 0;i<3;i++)
				position[i] -= pos.position[i];
			for (int i = 0;i<3;i++)
				orientation[i] -= pos.orientation[i];
		};

		void operator*=(const float x) {
			for (int i = 0;i<3;i++)
				position[i] *= x;
			for (int i = 0;i<3;i++)
				orientation[i] *= x;
		};
		void operator/=(const float x) {
			for (int i = 0;i<3;i++)
				position[i] /= x;
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



	HomVector position;
	HomVector orientation;

	rational gripperAngle;
};


#endif /* SPATIAL_H_ */

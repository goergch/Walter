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

using techsoft::mslice;
typedef techsoft::matrix<rational>  Matrix;
typedef techsoft::matrix<rational>  HomMatrix;
typedef std::valarray<rational> HomVector;
typedef std::valarray<rational> Vector;
typedef std::valarray<rational> JointAngleType;

class Pose {
	public:
		Pose() {
			orientation = { 0.0, 0.0, 0.0, 1.0};
			position 	= { 0.0, 0.0, 0.0, 1.0};
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

	HomVector position;
	HomVector orientation;

	rational gripperAngle;
};


#endif /* SPATIAL_H_ */

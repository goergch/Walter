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
		Pose(Pose& pose): Pose() {
			position = pose.position;
			orientation = pose.orientation;
		};
		Pose(HomVector& pPosition, HomVector& pOrientation) {
			position = pPosition;
			orientation = pOrientation;
		};

		void operator= (const Pose& pose) {
			position = pose.position;
			orientation = pose.orientation;
		}

	HomVector position;
	HomVector orientation;
};


#endif /* SPATIAL_H_ */

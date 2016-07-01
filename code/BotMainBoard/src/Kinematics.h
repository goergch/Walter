/*
 * Kinematics.h
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#ifndef KINEMATICS_H_
#define KINEMATICS_H_

class Kinematics {
public:
	Kinematics();

	static Kinematics& getInstance() {
			static Kinematics instance;
			return instance;
	}

};

#endif /* KINEMATICS_H_ */

/*
 * Kinematics.cpp
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include "Kinematics.h"
#include "Util.h"
#include "ActuatorProperty.h"


#define X 0
#define Y 1
#define Z 2

#define LOG_KIN_DETAILS false

Kinematics::Kinematics() {
}

JointAngles Kinematics::getNullPositionAngles() {
	return JointAngles::getDefaultPosition();
}



// setup denavit hardenberg parameters and set the
// rotation matrixes of the gripper to view coord.
void Kinematics::setup() {

	// define and compute Denavit Hardenberg Parameter
	// check Kinematics.xls for explantation
	DHParams[0] = DenavitHardenbergParams(radians(-90.0), 	0, 				HipHeight);
	DHParams[1] = DenavitHardenbergParams(0, 			  	UpperArmLength, 0);
	DHParams[2] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[3] = DenavitHardenbergParams(radians(90.0), 	0, 				TotalForearmLength);
	DHParams[4] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[5] = DenavitHardenbergParams(0, 				0, 				totalHandLength);

	// view has another coord system than the gripper, prepare the rotation matrices
	// otherwise, the roll/nick/yaw angles had no zero position of 0,0,0
	computeRotationMatrix(radians(-90), radians(-90), radians(-90), hand2View);

	// store the rotation matrix that converts the gripper to the view
	hand2View[0][3] = 0*-100;
	view2Hand = hand2View;
	view2Hand.inv();
}


// use DenavitHardenberg parameter and compute the Dh-Transformation matrix with a given joint angle (theta)
void Kinematics::computeDHMatrix(int actuatorNo, rational pTheta, float d, HomMatrix& dh) {

	rational ct = cos(pTheta);
	rational st = sin(pTheta);

	rational a = DHParams[actuatorNo].getA(); 		// length of the actuator
	rational sa = DHParams[actuatorNo].sinalpha();	// precomputed for performance (alpha is constant)
	rational ca = DHParams[actuatorNo].cosalpha();	// precomputed for performance (alpha is constant)

	dh = HomMatrix(4,4,
			{ ct, 	-st*ca,  st*sa,  a*ct,
			  st, 	 ct*ca, -ct*sa,	 a*st,
			  0,	 sa,		ca,		d,
			  0,	 0,		     0,		1});
}

// use DenavitHardenberg parameter and compute the DH-Transformation matrix with a given joint angle (theta)
// (used for joints besides the hand)
void Kinematics::computeDHMatrix(int actuatorNo, rational pTheta, HomMatrix& dh) {
	if (actuatorNo < HAND)
		computeDHMatrix(actuatorNo, pTheta, DHParams[actuatorNo].getD(), dh);
	else
		LOG(ERROR) << "computeDHMatrix called for HAND";
}

// real length of the hand depending on the gripper angle
float Kinematics::getHandLength(float gripperAngle) {
	return totalHandLength - GripperLeverLength*(1.0-cos(gripperAngle));
}

// compute forward kinematics, i.e. by given joint angles compute the
// position and orientation of the gripper center
void Kinematics::computeForwardKinematics(Pose& pose ) {
	// convert angles to intern offsets where required (angle 1)
	rational angle[NumberOfActuators] = {
			pose.angles[0],pose.angles[1]-radians(90),pose.angles[2],pose.angles[3],pose.angles[4],pose.angles[5],pose.angles[6] };

	// compute final position by multiplying all DH transformation matrixes
	HomMatrix current;
	HomMatrix currDHMatrix;
	computeDHMatrix(HIP, angle[HIP], current);

	computeDHMatrix(UPPERARM, angle[UPPERARM], currDHMatrix);
	current *= currDHMatrix;

	computeDHMatrix(FOREARM, angle[FOREARM], currDHMatrix);
	current *= currDHMatrix;

	computeDHMatrix(ELLBOW, angle[ELLBOW], currDHMatrix);
	current *= currDHMatrix;

	computeDHMatrix(WRIST, angle[WRIST], currDHMatrix);
	current *= currDHMatrix;

	computeDHMatrix(HAND, angle[HAND], getHandLength(angle[GRIPPER]), currDHMatrix);
	current *= currDHMatrix;

	// compute view from gripper matrix
	current *= hand2View;

	// position of hand is given by last row of transformation matrix
	pose.position = current.column(3);

	// compute orientations out of homogeneous transformation matrix
	// (as given in https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel)
	rational beta = atan2(-current[2][0], sqrt(current[0][0]*current[0][0] + current[1][0]*current[1][0]));
	rational gamma = 0;
	rational alpha = 0;
	if (almostEqual(beta, HALF_PI, floatPrecision)) {
		alpha = 0;
		gamma = atan2(current[0][1], current[1][1]);
	} else {
			if (almostEqual(beta, -HALF_PI,floatPrecision)) {
				alpha = 0;
				gamma = -atan2(current[0][1], current[1][1]);
			} else {
				alpha = atan2(current[1][0],current[0][0]);
				gamma = atan2(current[2][1], current[2][2]);
			}
	}

	// assign to return param
	pose.orientation[0] = gamma;
	pose.orientation[1] = beta;
	pose.orientation[2] = alpha;
	pose.gripperAngle = pose.angles[GRIPPER];

	LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(5) << "angles=("
			<< pose.angles[0] << ", " << pose.angles[1] << ", " << pose.angles[2] << ", "
			<< pose.angles[3] << ", " << pose.angles[4] << ", " << pose.angles[5] << ", " << pose.angles[6] << ")=("
			<< degrees(pose.angles[0]) << ", " << degrees(pose.angles[1]) << ", " << degrees(pose.angles[2]) << ", "
			<< degrees(pose.angles[3]) << ", " << degrees(pose.angles[4]) << ", " << degrees(pose.angles[5]) << ", " << degrees(pose.angles[6]) << ")"
			<< " Pose="
			<< "{(" << pose.position[0] << "," << pose.position[1] << "," << pose.position[2] << ");("
			<< pose.orientation[0] << "," << pose.orientation[1] << "," << pose.orientation[2] << "|" << pose.gripperAngle << ")}";
}


// compute reverse kinematics, i.e. compute angles out of pose
void Kinematics::computeInverseKinematicsCandidates(const Pose& tcp, const JointAngles& current, std::vector<KinematicsSolutionType> &solutions) {
	LOG_IF(LOG_KIN_DETAILS,DEBUG)  << setprecision(4)
			<< "{TCP=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << "|" << tcp.gripperAngle << ")})";

	// 1. Step compute angle0 (base)
	// - compute Transformationsmatrix T0-6 out of tool centre point (tcp)
	// - translate tcp in the direction of tcp's orientation by - wristlength -> wrist centre point (wcp)
	// - compute angle0 by arctan of wcp's projection to the floor
	// results in two possible solutions, called forward solution and backward solution

	// start with T0-6
	rational sinx = sin(tcp.orientation[X]);
	rational cosx = cos(tcp.orientation[X]);
	rational siny = sin(tcp.orientation[Y]);
	rational cosy = cos(tcp.orientation[Y]);
	rational sinz = sin(tcp.orientation[Z]);
	rational cosz = cos(tcp.orientation[Z]);

	// Set transformation matrix T06
	// left upper 3x3 part is rotation matrix out of three euler angles in zy'x'' model
	// (http://www-home.htwg-konstanz.de/~bittel/ain_robo/Vorlesung/02_PositionUndOrientierung.pdf)
	// (actually only columns 3 and 4 are required, but compute everything for debugging)
	HomMatrix T06 = HomMatrix(4,4,
		{ 	cosz*cosy,	cosz*siny*sinx-sinz*cosx,	cosz*siny*cosx+sinz*sinx,	tcp.position[0],
			sinz*cosy,	sinz*siny*sinx+cosz*cosx,	sinz*siny*cosx-cosz*sinx,	tcp.position[1],
			-siny,		cosy*sinx,					cosy*cosx,					tcp.position[2],
			0,			0,							0,							1 });

	// transform transformation matrix to reflect the gripper matrix instead of the view matrix
	T06 *= view2Hand;
	// compute wcp from tcp's perspective, then via T06 from world coord
	HomVector wcp_from_tcp_perspective = { 0,0,-getHandLength(tcp.gripperAngle),1 };
	HomVector wcp = T06 * wcp_from_tcp_perspective;

	// compute base angle by wrist position
	// we have two possible solutions, looking forward and looking backward
	// depending on the sign of the tcp x-coordinate, we assign the two solutions to
	// the forward-angle and the backward-angle
	rational angle0_solution1 = atan2(wcp[Y], wcp[X]);
	rational angle0_solution2 = atan2(-wcp[Y], -wcp[X]);

	// singularity check: if we are right above the origin, take the current angle
	if ((fabs(wcp[Y]) < floatPrecision) &&  (fabs(wcp[X]) < floatPrecision)) {
		angle0_solution1 = current[0];
		angle0_solution2 = HALF_PI - current[0];
	}

	rational angle0_forward = 0;
	rational angle0_backward = 0;
	bool tcpXPositive = tcp.position[X] >= 0;

	if (tcpXPositive) {
		angle0_forward =  angle0_solution1;
		angle0_backward = angle0_solution2;
	} else {
		angle0_forward =  angle0_solution2;
		angle0_backward = angle0_solution1;
	}
	LOG_IF(LOG_KIN_DETAILS,DEBUG) << "angle0_forward=" << angle0_forward << " angle0_backward=" << angle0_backward;

	// 2. Compute angle1 and angle2
	// use triangle of joint1(A=base + baseheight),joint2(C),and joint3(WCP)
	// - triangle sides are a=forearmlength, b=upperarmlength, c=distance(wcp, base)
	// - compute angles with cosinus sentence

	// distance of joint 1 and wcp
	rational z_distance_joint1_wcp = wcp[Z]-HipHeight;
	// distance of base and wcp when looked at from top
	rational distance_base_wcp_from_top = hypothenuseLength(wcp[X],wcp[Y]);
	// side c of the triangle
	rational c = hypothenuseLength(z_distance_joint1_wcp,distance_base_wcp_from_top);
	rational b = UpperArmLength;
	rational a = TotalForearmLength;
	rational alpha = triangleAlpha(a,b,c);
	rational gamma = triangleGamma(a,b,c);

	// flip flags states whether triangle is in non-flipping or flipping position
	rational flipFlag_forward = tcpXPositive?1.0:-1.0;
	rational flipFlag_backward = tcpXPositive?-1.0:1.0;

	rational delta_forward = atan2(z_distance_joint1_wcp, flipFlag_forward*distance_base_wcp_from_top);
	rational delta_backward = atan2(z_distance_joint1_wcp, flipFlag_backward*distance_base_wcp_from_top);

	rational angle1_forward_sol1 = HALF_PI - ( delta_forward + alpha);
	rational angle1_forward_sol2 = HALF_PI - ( delta_forward - alpha);
	rational angle1_backward_sol1 = HALF_PI - ( delta_backward + alpha);
	rational angle1_backward_sol2 = HALF_PI - ( delta_backward - alpha);

	rational angle2_sol2 = gamma - (PI*3.0/2.0);
	rational angle2_sol1 = HALF_PI - gamma;

	LOG_IF(LOG_KIN_DETAILS,DEBUG) << "triangle (a,b,c)=(" << setprecision(5) << a << "," << b << "," << c << ")"
			<< "alpha=" << alpha << " gamma=" << gamma
			<< "angle1_fwd_1= " << angle1_forward_sol1
			<< "angle1_fwd_2= " << angle1_forward_sol2
			<< "angle1_bck_1= " << angle1_backward_sol1
			<< "angle1_bck_2= " << angle1_backward_sol2
			<< "angle2_1= " << angle2_sol1
			<< "angle2_2= " << angle2_sol2;

	// initialize all possible 8 solutions
	solutions.resize(8);
	for (int i = 0;i<8;i++) {
		// solutions[i].angles.resize(NumberOfActuators);
		solutions[i].angles.null();
		solutions[i].angles[GRIPPER] = tcp.gripperAngle;
	}

	// 3. compute angle3, angle4, angle5
	// - compute rotation matrix R0-3
	// - compute Inverse(R0-3), ( which equals the transposed matrix)
	// - take R0-6 aus of T0-6(which we already have)
	// - derive R3-6 by inverse(R0-3)*R0-6
	// - compute angle3,4,5 by solving R3-6

	computeIKUpperAngles(tcp, current, PoseConfigurationType::PoseDirectionType::FRONT, PoseConfigurationType::PoseFlipType::NO_FLIP,
			angle0_forward, angle1_forward_sol1, angle2_sol1, T06,	solutions[0], solutions[1]);

	computeIKUpperAngles(tcp, current, PoseConfigurationType::PoseDirectionType::FRONT, PoseConfigurationType::PoseFlipType::FLIP,
			angle0_forward, angle1_forward_sol2, angle2_sol2, T06, solutions[2], solutions[3]);

	computeIKUpperAngles(tcp, current, PoseConfigurationType::PoseDirectionType::BACK, PoseConfigurationType::PoseFlipType::NO_FLIP,
			angle0_backward, angle1_backward_sol1, angle2_sol1, T06,	solutions[4], solutions[5]);

	computeIKUpperAngles(tcp, current, PoseConfigurationType::PoseDirectionType::BACK, PoseConfigurationType::PoseFlipType::FLIP,
			angle0_backward, angle1_backward_sol2, angle2_sol2, T06, solutions[6], solutions[7]);

}

void Kinematics::computeIKUpperAngles(
		const Pose& tcp, const JointAngles& current, PoseConfigurationType::PoseDirectionType poseDirection, PoseConfigurationType::PoseFlipType poseFlip,
		rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
		KinematicsSolutionType &sol_up, KinematicsSolutionType &sol_down) {

	/*
	LOG(DEBUG) << setprecision(4)
			<< "{p=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << "|" << tcp.gripperAngle << ")})" << endl
			<< "poseDirection=" << poseDirection << " poseFlip=" << poseFlip
			<< "angle0=" << angle0 << " angle1=" << angle1 << " angle2=" << angle2;
*/
	sol_up.config.poseFlip = poseFlip;
	sol_up.config.poseDirection = poseDirection;
	sol_up.config.poseTurn= PoseConfigurationType::UP;

	sol_up.angles[0] = angle0;
	sol_up.angles[1] = angle1;
	sol_up.angles[2] = angle2;

	sol_down.config.poseFlip = poseFlip;
	sol_down.config.poseDirection = poseDirection;
	sol_down.config.poseTurn= PoseConfigurationType::DOWN;

	sol_down.angles[0] = angle0;
	sol_down.angles[1] = angle1;
	sol_down.angles[2] = angle2;

	// 3. compute angle3, angle4, angle5
	// - compute rotation matrix R0-3
	// - compute Inverse(R0-3), ( which equals the transposed matrix)
	// - take R0-6 aus of T0-6(which we already have)
	// - derive R3-6 by inverse(R0-3)*R0-6
	// - compute angle3,4,5 by solving R3-6
	HomMatrix T01, T12, T23;
	computeDHMatrix(0, angle0, T01);
	computeDHMatrix(1, angle1-radians(90), T12);
	computeDHMatrix(2, angle2, T23);

	Matrix R01 = T01[mslice(0,0,3,3)];
	Matrix R12 = T12[mslice(0,0,3,3)];
	Matrix R23 = T23[mslice(0,0,3,3)];
	Matrix R02 = R01*R12;
	Matrix R03 = R02*R23;

	// compute inverse by transposing manually
	Matrix R03_inv(R03);
	mswap(R03_inv[0][1], R03_inv[1][0]);
	mswap(R03_inv[0][2], R03_inv[2][0]);
	mswap(R03_inv[1][2], R03_inv[2][1]);

	Matrix R06 = T06[mslice(0,0,3,3)];
	Matrix R36 = R03_inv*R06;

	rational R36_22 = R36[2][2];
	rational R36_01 = R36[0][1];

	// sometimes, R36_22 is slightly greater than 1 due to floating point arithmetics
	// since we call acos afterwards, we need to compensate that.
	if ((fabs(R36_22) > 1.0d) && (fabs(R36_22) < (1.0d+floatPrecision))) {
		R36_22 = (R36_22>0)?1.0:-1.0;
	}

	if ((fabs(R36_01) > 1.0d) && (fabs(R36_01) < (1.0d+floatPrecision))) {
		R36_01 = (R36_01>0)?1.0:-1.0;
	}

	sol_up.angles[4]   = acos(R36_22);
	sol_down.angles[4] = -sol_up.angles[4];

	rational sin_angle4_1 = sin(sol_up.angles[4]);
	rational sin_angle4_2 = -sin_angle4_1;

	/*
	LOG(DEBUG) << setprecision(6) << endl
			<< "R01=" << R01;

	LOG(DEBUG) << setprecision(6) << endl
			<< "T12=" << T12;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R23=" << R23;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R03=" << R03;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R03_inv=" << R03_inv;
	LOG(DEBUG) << setprecision(6) << endl
			<< "R36=" << R36;
			*/


	// if wrist is 0°, there is an infinite number of solutions.
	// this requires a special treatment that keeps angles close to current position
	if (sqr(sin_angle4_1) < floatPrecision) {

		sol_up.angles[3]   = current[3];
		sol_down.angles[3] = current[3];

		rational asinR36_01 = asin(-R36_01);
		sol_up.angles[5]  = asinR36_01  - sol_up.angles[3];
		sol_down.angles[5]= asinR36_01  - sol_down.angles[3];

		/*
		LOG(DEBUG) << setprecision(4) << "BBB sol_up.angles[4]" << sol_up.angles[4] << " sol_up.angles[3]" << sol_up.angles[3] << "  sol_down.angles[5]" <<  sol_down.angles[5]
				<< "angle3_offset=" << sol_up.angles[3]-current[3] << "sol_up.angles[3]end=" << sol_up.angles[3] - (sol_up.angles[3]-current[3])
				<< " sol_up.angles[5].end=" << sol_down.angles[5] + (sol_up.angles[3]-current[3]) << " R36_22" << R36_22;
		 */

        // normalize angles by adding or substracting PI to bring it in an interval -PI..PI
        while ((abs( sol_up.angles[5] - current[5]) >
             abs( sol_up.angles[5] + PI - current[5])) &&
        	(sol_up.angles[5] + PI <= actuatorConfigType[5].maxAngle)) {
        	sol_up.angles[5]   += PI;
        	sol_down.angles[5] += PI;
        }
       	while ((abs( sol_up.angles[5] - current[5]) >
             abs( sol_up.angles[5] - PI - current[5])) &&
        	(sol_up.angles[5] - PI >= actuatorConfigType[5].minAngle)) {
       		sol_up.angles[5]   -= PI;
            sol_down.angles[5] -= PI;
        }
        LOG(DEBUG) << setprecision(4) << "BBB sol_up.angles[5]" << sol_up.angles[5] << " current[5]]" << current[5];
	}
	else {
		/*
		LOG(DEBUG) << setprecision(4) << "AAA sin_angle_4_1" << sin_angle4_1 << " sin_angle_4_2" << sin_angle4_2
					<< "R36_22=" << R36_22 << "R36[2][1]=" << R36[2][1] << "R36[2][0]=" << R36[2][0] << "R36[1][2]=" << R36[1][2] << " R36[0][2]=" << R36[0][2];
		*/
		sol_up.angles[5]   = atan2( - R36[2][1]/sin_angle4_1, R36[2][0]/sin_angle4_1);
		sol_down.angles[5] = atan2( - R36[2][1]/sin_angle4_2, R36[2][0]/sin_angle4_2);

		sol_up.angles[3]   = -atan2( R36[1][2]/sin_angle4_1,- R36[0][2]/sin_angle4_1);
		sol_down.angles[3] = -atan2( R36[1][2]/sin_angle4_2,- R36[0][2]/sin_angle4_2);

   		// LOG(DEBUG) << setprecision(4) << "CCCB sol_up.angles[5]" << sol_up.angles[5] << " current[5]]" << current[5];
	}

	/*
	LOG(DEBUG) << setprecision(4) << endl
				<<  "solup[" << sol_up.config.poseDirection<< "," << sol_up.config.poseFlip  << "," << sol_up.config.poseTurn<< "]=("
					<< sol_up.angles[0] << "," << sol_up.angles[1] << ","<< sol_up.angles[2] << ","<< sol_up.angles[3] << ","<< sol_up.angles[4] << ","<< sol_up.angles[5] << ")=("
					<< degrees(sol_up.angles[0]) << "," << degrees(sol_up.angles[1]) << ","<< degrees(sol_up.angles[2]) << ","<< degrees(sol_up.angles[3]) << ","<< degrees(sol_up.angles[4]) << ","<< degrees(sol_up.angles[5]) << ")" << endl
				<< "soldn[" << sol_down.config.poseDirection << "," << sol_down.config.poseFlip << "," << sol_down.config.poseTurn<< "]=("
					<< sol_down.angles[0] << "," << sol_down.angles[1] << ","<< sol_down.angles[2] << ","<< sol_down.angles[3] << ","<< sol_down.angles[4] << ","<< sol_down.angles[5] << ")=("
					<< degrees(sol_down.angles[0]) << "," << degrees(sol_down.angles[1]) << ","<< degrees(sol_down.angles[2]) << ","<< degrees(sol_down.angles[3]) << ","<< degrees(sol_down.angles[4]) << ","<< degrees(sol_down.angles[5]) << ")" << endl;
					*/
}

// Double check if the solution has the same value like a forward computation.
// For testing/debugging purposes.
bool Kinematics::isSolutionValid(const Pose& pose, const KinematicsSolutionType& sol, rational &precision) {
	Pose computedPose;
	computedPose.angles = sol.angles;
	computeForwardKinematics(computedPose);

	rational maxDistance = sqr(0.1f); // 1mm deviation is allowed
	rational poseDistance = sqr(computedPose.position[X] - pose.position[X]) +
							sqr(computedPose.position[Y] - pose.position[Y]) +
							sqr(computedPose.position[Z] - pose.position[Z]);

	rational maxAngle= sqr(radians(0.1f)); // 0.1° deviation is allowed
	rational nickDistance = sqr(computedPose.orientation[0] - pose.orientation[0]);
	// when checking the orientation, turning by 180° gives the same orientation
	while (nickDistance >= PI-floatPrecision)
		nickDistance -= PI;
	while (nickDistance <= -PI+floatPrecision)
		nickDistance += PI;

	// the other angles do not need to be checked, since this is contained in the position
	precision = poseDistance + nickDistance;
	bool isEqual = (poseDistance < maxDistance) && (sqr(nickDistance) < maxAngle);
	return isEqual;
}

float Kinematics::anglesDistance(const JointAngles& angleSet1, const JointAngles& angleSet2) {
	rational distance = 0.0;
	for (int i = 0;i<7;i++)
		distance += sqr(angleSet1[i] - angleSet2[i]);
	return distance;
}

float Kinematics::getAngularSpeed(rational angle1, rational angle2, int timeDiff_ms) {
	return (angle1-angle2)*1000.0/(float(timeDiff_ms));
}

float Kinematics::getAngularAcceleration(rational angle1, rational angle2, rational angle3, int timeDiff_ms) {
	float speed1 = getAngularSpeed(angle1, angle2, timeDiff_ms);
	float speed2 = getAngularSpeed(angle2, angle3, timeDiff_ms);
	return (speed1-speed2)*1000.0/(float(timeDiff_ms));
}


float Kinematics::maxAcceleration(const JointAngles& angleSet1, const JointAngles& angleSet2,  const JointAngles& angleSet3, int timeDiff_ms, int& jointNo) {
	float maxAcc = 0.0;
	for (int i = 0;i<7;i++) {
		float acc = getAngularAcceleration(angleSet1[i],angleSet2[i], angleSet3[i], timeDiff_ms) / (actuatorConfigType[i].maxAcc *(360/ 60.0)/actuatorConfigType[i].gearRatio);
		if (fabs(acc) > fabs(maxAcc)) {
			maxAcc = acc;
			jointNo = i;
		}
	}
	return maxAcc;
}

float Kinematics::maxSpeed(const JointAngles& angleSet1, const JointAngles& angleSet2, int timeDiff_ms, int&jointNo) {
	float maxSeed= 0.0;
	for (int i = 0;i<7;i++) {
		float speed = getAngularSpeed(angleSet1[i],angleSet2[i], timeDiff_ms) / (actuatorConfigType[i].maxSpeed*(360.0/60.0)/actuatorConfigType[i].gearRatio);
		if (fabs(speed) > fabs(maxSeed)){
			maxSeed= speed;
			jointNo = i;
		}
	}
	return maxSeed;
}

bool Kinematics::isIKInBoundaries( const KinematicsSolutionType &sol, int& actuatorOutOfBounds) {
	bool ok = true;
	for (int i = 0;i<sol.angles.size();i++) {
		if ((sol.angles[i] < (actuatorConfigType[i].minAngle-floatPrecision)) || (sol.angles[i] > (actuatorConfigType[i].maxAngle+floatPrecision))) {
			actuatorOutOfBounds = i;
			ok = false;
		}
	}
	return ok;
}


// select the solution that is best in terms of little movement
bool Kinematics::chooseIKSolution(const JointAngles& current, const Pose& pose, std::vector<KinematicsSolutionType> &solutions,
								  int &choosenSolution, std::vector<KinematicsSolutionType>& validSolutions) {
	rational bestDistance = 0;
	choosenSolution = -1;
	validSolutions.clear();
	for (unsigned i = 0;i<solutions.size();i++ ) {
		const KinematicsSolutionType& sol = solutions[i];
		// check only valid solutions
		rational precision;
		if (isSolutionValid(pose,sol, precision)) {
			// check if in valid boundaries
			int actuatorOutOfBound;
			if (isIKInBoundaries(sol, actuatorOutOfBound)) {
				validSolutions.insert(validSolutions.end(),sol);
				// check if solution is close the current situation
				rational distance = 0.0f;
				for (unsigned j = 0;j< Actuators;j++)
					distance +=	sqr(sol.angles[j] - current[j]);
				if ((distance < bestDistance) || (choosenSolution == -1)) {
					choosenSolution = i;
					bestDistance = distance;
				}
				LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(4)<< endl
							<< "solution[" << i << "] ok!(" << distance << ") [" << sol.config.poseDirection << "," << sol.config.poseFlip << "," << sol.config.poseTurn<< "]=("
								<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
								<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")"
								<< "curr=(" << degrees(current[0]) << "," << degrees(current[1]) << ","<< degrees(current[2]) << ","<< degrees(current[3]) << ","<< degrees(current[4]) << ","<< degrees(current[5]) << ")" << endl;
			}
			else {
				KinematicsSolutionType sol = solutions[i];
				LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(4)<< endl
							<< "solution[" << i << "] out of bounds!(" << actuatorOutOfBound << ") [" << sol.config.poseDirection << "," << sol.config.poseFlip << "," << sol.config.poseTurn<< "]=("
								<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
								<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;

			}
		} else {
			KinematicsSolutionType sol = solutions[i];
			LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(4)<< endl
						<< "solution[" << i << "] invalid(" << precision << ")! [" << sol.config.poseDirection << "," << sol.config.poseFlip << "," << sol.config.poseTurn<< "]=("
							<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
							<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
		}
	}

	if ((choosenSolution >= 0)) {
		KinematicsSolutionType sol = solutions[choosenSolution];
		LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(4)<< endl
					<< "best solution [" <<  choosenSolution << "]" << sol.config.poseDirection<< "," << sol.config.poseFlip  << "," << sol.config.poseTurn<< "]=("
						<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
						<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
	}
	return (choosenSolution >= 0);
}

// There are some poses (e.g. when axis point into the same direction) that give
// a strange behaviour due to floating point imprecision. This behaviour is
// next to poles when several solutions are possible. These poles should be
// avoided in trajectories.
// The following functions slightly moves a number if it is close to such a pole
/*
rational avoidPole(rational x, rational pole, rational deviation) {
	if (fabs(x-pole) < deviation) {
		if (x > 0)
			return pole + deviation;
		else
			return -pole - deviation;
	}
	else
		return x;
}*/


bool Kinematics::computeInverseKinematics(Pose& pose) {

	KinematicsSolutionType solution;
	std::vector<KinematicsSolutionType> validSolutions;

	bool ok = Kinematics::getInstance().computeInverseKinematics(pose, solution,validSolutions);
	if (ok)
		pose.angles = solution.angles;
	return ok;

}


bool Kinematics::computeInverseKinematics(
		const Pose& pose, KinematicsSolutionType &solution, std::vector<KinematicsSolutionType> &validSolution ) {
	std::vector<KinematicsSolutionType> solutions;

	// avoid pole position when kinematics shows strange behaviour
	// move the position/orientation slightly around these poles (by 0.0000001 mm)
	Pose poseWithoutPoles = pose;

	computeInverseKinematicsCandidates(poseWithoutPoles, pose.angles, solutions);
	int selectedIdx = -1;
	bool ok = chooseIKSolution(pose.angles, poseWithoutPoles, solutions, selectedIdx, validSolution);
	if (ok) {
		solution = solutions[selectedIdx];
		KinematicsSolutionType sol = solution;

		LOG_IF(LOG_KIN_DETAILS,DEBUG) << setprecision(4)<< endl
					<< "solution found [" << sol.config.poseDirection << "," << sol.config.poseFlip << "," << sol.config.poseTurn<< "]=("
						<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
						<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;

	} else {
		LOG(ERROR) << "no solution found";
	}
	return ok;
}

PoseConfigurationType Kinematics::computeConfiguration(const JointAngles angles) {
	PoseConfigurationType config;
	config.poseDirection = (abs(degrees(angles[HIP]))<= 90)?PoseConfigurationType::FRONT:PoseConfigurationType::BACK;
	config.poseFlip = (degrees(angles[FOREARM])<-90.0f)?PoseConfigurationType::FLIP:PoseConfigurationType::NO_FLIP;
	config.poseTurn = (degrees(angles[ELLBOW])< 0.0f)?PoseConfigurationType::UP:PoseConfigurationType::DOWN;
	return config;
}

void Kinematics::computeRotationMatrix(rational x, rational y, rational z, HomMatrix& m) {

	rational sinA = sin(x);
	rational cosA = cos(x);
	rational sinB = sin(y);
	rational cosB = cos(y);
	rational sinC = sin(z);
	rational cosC = cos(z);

	m = HomMatrix(4,4,
			{ 	cosC*cosB, 	-sinC*cosA+cosC*sinB*sinA,  	sinC*sinA+cosC*sinB*cosA, 	0,
				sinC*cosB, 	 cosC*cosA + sinC*sinB*sinA, 	cosC*sinA+sinC*sinB*cosA, 	0,
				-sinB,	 	cosB*sinA,						cosB*cosA,					0,
				0,			0,								0,							1});
}


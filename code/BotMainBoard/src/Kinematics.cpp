/*
 * Kinematics.cpp
 *
 *  Created on: 27.06.2016
 *      Author: JochenAlt
 */

#include "setup.h"
#include "Kinematics.h"
#include "Util.h"

#define X 0
#define Y 1
#define Z 2

Kinematics::Kinematics() {
	isSetup = false;
}


void Kinematics::setup() {

	// define and precompute Denavit Hardenberg Parameter
	// check Kinematics.xls with kinematics documentation
	DHParams[0] = DenavitHardenbergParams(radians(-90.0), 	0, 				HipHeight);
	DHParams[1] = DenavitHardenbergParams(0, 			  	UpperArmLength, 0);
	DHParams[2] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[3] = DenavitHardenbergParams(radians(90.0), 	0, 				ForearmLength);
	DHParams[4] = DenavitHardenbergParams(radians(-90.0), 	0, 				0);
	DHParams[5] = DenavitHardenbergParams(0, 				0, 				HandLength);
	isSetup = true;
}


// use DenavitHardenberg parameter and compute the Dh-Transformation matrix with a given joint angle (theta)
void Kinematics::computeDHMatrix(const DenavitHardenbergParams& DHparams, rational pTheta, HomMatrix& dh) {

	rational ct = cos(pTheta);
	rational st = sin(pTheta);

	rational a = DHparams.getA();
	rational d = DHparams.getD();

	rational sa = DHparams.sinalpha();
	rational ca = DHparams.cosalpha();

	dh = HomMatrix(4,4,
			{ ct, 	-st*ca,  st*sa,  a*ct,
			  st, 	 ct*ca, -ct*sa,	 a*st,
			  0,	 sa,		ca,		d,
			  0,	 0,		     0,		1});
}

// compute forward kinematics, i.e. by given joint angles compute the
// position and orientation of the gripper center
void Kinematics::computeForwardKinematics(const JointAngleType pAngle, Pose& pose ) {

	// convert angles first
	rational angle[Actuators];
	angle[0] = pAngle[0];
	angle[1] = pAngle[1]-radians(90);
	angle[2] = pAngle[2];
	angle[3] = pAngle[3];
	angle[4] = pAngle[4];
	angle[5] = pAngle[5];

	HomMatrix current;
	computeDHMatrix(DHParams[0], angle[0], current);
	// LOG(DEBUG) << "current " << endl << setprecision(4) << current;

	for (int i = 1;i<=5;i++) {
		HomMatrix currDHMatrix;
		computeDHMatrix(DHParams[i], angle[i], currDHMatrix);

		current *= currDHMatrix;
	}

	// LOG(DEBUG) << "forward transformation" << endl << setprecision(4) << current;

	// position of hand is given by last row of transformation matrix
	pose.position = current.column(3);

	// compute orientations out of homogeneous transformation matrix
	// (as given in https://de.wikipedia.org/wiki/Roll-Nick-Gier-Winkel)
	rational beta = atan2(-current[2][0], sqrt(current[0][0]*current[0][0] + current[1][0]*current[1][0]));
	rational gamma = 0;
	rational alpha = 0;
	rational precision = 0.001f; // = differs by 0.1%
	if (almostEqual(beta, HALF_PI, precision)) {
		alpha = 0;
		gamma = atan2(current[0][1], current[1][1]);
	} else {
			if (almostEqual(beta, -HALF_PI,precision)) {
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
	pose.gripperAngle = pAngle[GRIPPER];

	LOG(DEBUG) << setprecision(4) << "angles=("
			<< pAngle[0] << ", " << pAngle[1] << ", " << pAngle[2] << ", "
			<< pAngle[3] << ", " << pAngle[4] << ", " << pAngle[5] << ", " << pAngle[6] << ")" << endl

			<< "TCP=" << setprecision(4)
			<< "{(" << pose.position[0] << "," << pose.position[1] << "," << pose.position[2] << ");("
			<< pose.orientation[0] << "," << pose.orientation[1] << "," << pose.orientation[2] << "|" << pose.gripperAngle << ")}";
}


// compute reverse kinematics, i.e. by position and orientation compute the
// angles of the joints
void Kinematics::computeInverseKinematicsCandidates(const Pose& tcp, std::vector<KinematicsSolutionType> &solutions) {
	LOG(DEBUG) << setprecision(4)
			<< "{TCP=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << "|" << tcp.gripperAngle << ")})";


	// 1. Step compute angle0 (base)
	// - compute Transformationsmatrix T0-6 out of tool centre point (tcp) pose
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

	// LOG(DEBUG) << setprecision(4) << endl << "T06=" << T06;

	HomVector wcp_from_tcp_perspective = { 0,0,-HandLength,1 };
	HomVector wcp = T06 * wcp_from_tcp_perspective;

	LOG(DEBUG) << setprecision(4) << endl << "WCP=(" << wcp[0] << "," << wcp[1] << "," << wcp[2] << ")";

	// compute base angle by wrist position
	// we have two possible solutions, looking forward and looking backward
	// depending on the sign of the tcp x-coordinate, we assign the two solutions to
	// the forward-angle and the backward-angle
	rational angle0_solution1 = atan2(wcp[Y], wcp[X]);
	rational angle0_solution2 = atan2(-wcp[Y], -wcp[X]);
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
	LOG(DEBUG) << "angle0_forward=" << angle0_forward << " angle0_backward=" << angle0_backward;

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
	rational a = ForearmLength;
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

	LOG(DEBUG) << "triangle (a,b,c)=(" << setprecision(5) << a << "," << b << "," << c << ")"
			<< "alpha=" << alpha << " gamma=" << gamma
			<< "angle1_fwd_1= " << angle1_forward_sol1
			<< "angle1_fwd_2= " << angle1_forward_sol2
			<< "angle1_bck_1= " << angle1_backward_sol1
			<< "angle1_bck_2= " << angle1_backward_sol2
			<< "angle2_1= " << angle2_sol1
			<< "angle2_2= " << angle2_sol2;

	solutions.resize(8);
	for (int i = 0;i<8;i++) {
		solutions[i].angles.resize(NumberOfActuators);
		solutions[i].angles[GRIPPER] = tcp.gripperAngle;
	}

	// 3. compute angle3, angle4, angle5
	// - compute rotation matrix R0-3
	// - compute Inverse(R0-3), ( which equals the transposed matrix)
	// - take R0-6 aus of T0-6(which we already have)
	// - derive R3-6 by inverse(R0-3)*R0-6
	// - compute angle3,4,5 by solving R3-6

	computeIKUpperAngles(tcp, KinematicConfigurationType::PoseDirectionType::FRONT, KinematicConfigurationType::PoseFlipType::NO_FLIP,
			angle0_forward, angle1_forward_sol1, angle2_sol1, T06,	solutions[0], solutions[1]);

	computeIKUpperAngles(tcp, KinematicConfigurationType::PoseDirectionType::FRONT, KinematicConfigurationType::PoseFlipType::FLIP,
			angle0_forward, angle1_forward_sol2, angle2_sol2, T06, solutions[2], solutions[3]);

	computeIKUpperAngles(tcp, KinematicConfigurationType::PoseDirectionType::BACK, KinematicConfigurationType::PoseFlipType::NO_FLIP,
			angle0_backward, angle1_backward_sol1, angle2_sol1, T06,	solutions[4], solutions[5]);

	computeIKUpperAngles(tcp, KinematicConfigurationType::PoseDirectionType::BACK, KinematicConfigurationType::PoseFlipType::FLIP,
			angle0_backward, angle1_backward_sol2, angle2_sol2, T06, solutions[6], solutions[7]);

}

void Kinematics::computeIKUpperAngles(
		const Pose& tcp, KinematicConfigurationType::PoseDirectionType poseDirection, KinematicConfigurationType::PoseFlipType poseFlip,
		rational angle0, rational angle1, rational angle2, const HomMatrix &T06,
		KinematicsSolutionType &sol_up, KinematicsSolutionType &sol_down) {

	LOG(DEBUG) << setprecision(4)
			<< "{p=(" << tcp.position[0] << "," << tcp.position[1] << "," << tcp.position[2] << ");("
			<< tcp.orientation[0] << "," << tcp.orientation[1] << "," << tcp.orientation[2] << "|" << tcp.gripperAngle << ")})" << endl
			<< "poseDirection=" << poseDirection << " poseFlip=" << poseFlip
			<< "angle0=" << angle0 << " angle1=" << angle1 << " angle2=" << angle2;

	sol_up.angles.resize(NumberOfActuators);
	sol_down.angles.resize(NumberOfActuators);

	sol_up.config.poseFlip = poseFlip;
	sol_up.config.poseDirection = poseDirection;
	sol_up.config.poseTurn= KinematicConfigurationType::UP;

	sol_up.angles[0] = angle0;
	sol_up.angles[1] = angle1;
	sol_up.angles[2] = angle2;

	sol_down.config.poseFlip = poseFlip;
	sol_down.config.poseDirection = poseDirection;
	sol_down.config.poseTurn= KinematicConfigurationType::DOWN;

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
	computeDHMatrix(DHParams[0], angle0, T01);
	computeDHMatrix(DHParams[1], angle1-radians(90), T12);
	computeDHMatrix(DHParams[2], angle2, T23);

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

	sol_up.angles[4]   = acos(R36[2][2]);
	sol_down.angles[4] = -sol_up.angles[4];

	rational sin_angle4_1 = sin(sol_up.angles[4]);
	rational sin_angle4_2 = -sin_angle4_1;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R01=" << R01;

	LOG(DEBUG) << setprecision(6) << endl
			<< "T12=" << T12;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R23=" << R23;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R03=" << R03;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R03_inv=" << R03;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R06=" << R06;

	LOG(DEBUG) << setprecision(6) << endl
			<< "R36=" << R36;

	double precision = 0.0001;
	if (abs(sin_angle4_1) < precision) {
		sol_up.angles[5]   = atan2(- R36[2][1], R36[2][0]);
		sol_down.angles[5] = sol_up.angles[5];
	}
	else {
		sol_up.angles[5]   = atan2( - R36[2][1]/sin_angle4_1, R36[2][0]/sin_angle4_1);
		sol_down.angles[5] = sol_up.angles[5];
	}

	if (abs(sin_angle4_1) < precision) {
		sol_up.angles[3]   = -atan2( R36[1][2], - R36[0][2]);
		sol_down.angles[3] = sol_up.angles[3];
	}
	else {
		sol_up.angles[3]   = -atan2( R36[1][2]/sin_angle4_1,- R36[0][2]/sin_angle4_1);
		sol_down.angles[3] = -atan2( R36[1][2]/sin_angle4_2,- R36[0][2]/sin_angle4_2);
	}


	KinematicsSolutionType sol = sol_up;
	LOG(DEBUG) << setprecision(4) << endl
				<<  "solup[" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
					<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
					<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
	sol = sol_down;
	LOG(DEBUG) << setprecision(4)<< endl
				<< "soldn[" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
					<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
					<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
}

void Kinematics::logSolution(string prefix, const KinematicsSolutionType& sol) {
	LOG(DEBUG) << setprecision(4)<< endl
				<< prefix << "[" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
					<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
					<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
}

bool Kinematics::isIKValid(const Pose& pose, const KinematicsSolutionType& sol) {
	Pose computedPose;
	computeForwardKinematics(sol.angles,computedPose);
	rational maxDistance = 2.0f; // inverse kinematics may differ from real one by 0.1mm
	rational distance = sqr(computedPose.position[X] - pose.position[X]) +
						sqr(computedPose.position[Y] - pose.position[Y]) +
						sqr(computedPose.position[Z] - pose.position[Z]);

	bool isEqual = (distance < sqr(maxDistance));
	LOG(DEBUG) << setprecision(4)<< endl
				<<  "[" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
					<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
					<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")"
					<< "distance=" << distance << endl;

	return isEqual;
}

bool Kinematics::isIKInBoundaries(ActuatorLimitsType limits, const KinematicsSolutionType &sol) {
	bool ok = true;
	for (unsigned i = 0;i<sol.angles.size();i++) {
		if ((sol.angles[i] < limits[i].minAngle) || (sol.angles[i] > limits[0].maxAngle))
			ok = false;
	}
	return ok;
}


bool Kinematics::chooseIKSolution(ActuatorLimitsType limits, JointAngleType current, const Pose& pose, std::vector<KinematicsSolutionType> &solutions,
								  int &choosenSolution, std::vector<bool> &validSolutions) {
	rational bestDistance = 0;
	choosenSolution = -1;
	validSolutions.clear();
	validSolutions.resize(solutions.size());
	for (unsigned i = 0;i<solutions.size();i++ ) {
		validSolutions[i] = false;

		const KinematicsSolutionType& sol = solutions[i];
		// check only valid solutions
		if (isIKValid(pose,sol)) {
			// check if in valid boundaries
			if (isIKInBoundaries(limits, sol)) {
				validSolutions[i] = true;
				// check if solution is close the current situation
				rational distance = 0.0f;
				for (unsigned j = 0;j< Actuators;j++) {
					rational currentAngle = current[j];
					rational solutionAngle = sol.angles[j];
					distance +=	sqr(solutionAngle - currentAngle);
				}
				if ((distance < bestDistance) || (choosenSolution == -1)) {
					choosenSolution = i;
					bestDistance = distance;
				}
			}
			else {
				KinematicsSolutionType sol = solutions[i];
				LOG(INFO) << setprecision(4)<< endl
							<< "solution out of bounds! [" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
								<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
								<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;

			}
		} else {
			KinematicsSolutionType sol = solutions[i];
			LOG(ERROR) << setprecision(4)<< endl
						<< "solution invalid! [" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
							<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
							<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
		}
	}

	if ((choosenSolution >= 0)) {
		KinematicsSolutionType sol = solutions[choosenSolution];
		LOG(INFO) << setprecision(4)<< endl
					<< "best solution! [" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
						<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
						<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
	}
	return (choosenSolution >= 0);
}

bool Kinematics::computeInverseKinematics(
		ActuatorLimitsType limits, JointAngleType current,
		const Pose& pose, KinematicsSolutionType &solution, std::vector<KinematicConfigurationType> &validConfigurations ) {
	std::vector<KinematicsSolutionType> solutions;
	computeInverseKinematicsCandidates(pose, solutions);
	int selectedIdx = -1;
	std::vector<bool> validSolution;
	bool ok = chooseIKSolution(limits, current, pose, solutions, selectedIdx, validSolution);
	if (ok) {
		solution = solutions[selectedIdx];
		KinematicsSolutionType sol = solution;
		LOG(INFO) << setprecision(4)<< endl
					<< "solution out of bounds! [" << sol.config.poseFlip << "," << sol.config.poseDirection << "," << sol.config.poseTurn<< "]=("
						<< sol.angles[0] << "," << sol.angles[1] << ","<< sol.angles[2] << ","<< sol.angles[3] << ","<< sol.angles[4] << ","<< sol.angles[5] << ")=("
						<< degrees(sol.angles[0]) << "," << degrees(sol.angles[1]) << ","<< degrees(sol.angles[2]) << ","<< degrees(sol.angles[3]) << ","<< degrees(sol.angles[4]) << ","<< degrees(sol.angles[5]) << ")" << endl;
	}
	return ok;
}

void Kinematics::computeConfiguration(const JointAngleType angles, KinematicConfigurationType &config) {
	config.poseDirection = (abs(degrees(angles[HIP]))<= 90)?KinematicConfigurationType::FRONT:KinematicConfigurationType::BACK;
	config.poseFlip = (degrees(angles[FOREARM])<-90.0f)?KinematicConfigurationType::FLIP:KinematicConfigurationType::NO_FLIP;
	config.poseTurn = (degrees(angles[ELLBOW])< 0.0f)?KinematicConfigurationType::UP:KinematicConfigurationType::DOWN;
}

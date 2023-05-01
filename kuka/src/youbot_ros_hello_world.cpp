//
// Gaze form study kuka controller
//

#include "ros/ros.h"
#include "boost/units/systems/si.hpp"
#include "boost/units/io.hpp"
#include "brics_actuator/JointPositions.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/String.h"
#include "youbot_ros_hello_world/GameState.h"
#include "youbot_ros_hello_world/TimeState.h"
#include "youbot_ros_hello_world/GameUpdate.h"
#include <stdlib.h>
#include <time.h>
#include <math.h>

ros::Publisher armPublisher;
ros::Publisher gripperPublisher;
ros::Subscriber gameStateSubscriber;
ros::Subscriber timeStateSubscriber;
ros::Subscriber gameUpdateSubscriber;
bool jointval4 = true;
bool instructions = true;
void waveInFace(void);

// create a brics actuator message with the given joint position values
brics_actuator::JointPositions createArmPositionCommand(std::vector<double>& newPositions) {
	int numberOfJoints = 5;
	brics_actuator::JointPositions msg;

	if (newPositions.size() < numberOfJoints)
		return msg; // return empty message if not enough values provided

	for (int i = 0; i < numberOfJoints; i++) {
		// Set all values for one joint, i.e. time, name, value and unit
		brics_actuator::JointValue joint;
		joint.timeStamp = ros::Time::now();
		joint.value = newPositions[i];
		joint.unit = boost::units::to_string(boost::units::si::radian);

		// create joint names: "arm_joint_1" to "arm_joint_5" (for 5 DoF)
		std::stringstream jointName;
		jointName << "arm_joint_" << (i + 1);
		joint.joint_uri = jointName.str();

		// add joint to message
		msg.positions.push_back(joint);
	}

	return msg;
}

// create a brics actuator message for the gripper using the same position for both fingers
brics_actuator::JointPositions createGripperPositionCommand(double newPosition) {
	brics_actuator::JointPositions msg;

	brics_actuator::JointValue joint;
	joint.timeStamp = ros::Time::now();
	joint.unit = boost::units::to_string(boost::units::si::meter); // = "m"
	joint.value = newPosition;
	joint.joint_uri = "gripper_finger_joint_l";
	msg.positions.push_back(joint);		
	joint.joint_uri = "gripper_finger_joint_r";
	msg.positions.push_back(joint);		

	return msg;
}

double deg2rad(double degree)
{
    double pi = 3.14159265359;
    return (degree * (pi / 180));
}

double rad2deg(double rad)
{
	double pi = 3.14159265359;
	return (rad * 180 / pi);
}

void moveKukaArmJoints(double joint0, double joint1, double joint2, double joint3, double joint4) {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	jointvalues[0] = joint0;
	jointvalues[1] = joint1;
	jointvalues[2] = joint2;
	jointvalues[3] = joint3;
	jointvalues[4] = joint4;

	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
}

void standUpWaveHelper() {
	moveKukaArmJoints(deg2rad(70), 0.11, deg2rad(-100), deg2rad(190), 0.12);
	ros::Duration(1).sleep();
	moveKukaArmJoints(deg2rad(90), 0.11, deg2rad(-100), deg2rad(102), 0.12);
	ros::Duration(1).sleep();
}

// open and close gripper
void openGripper() {
	brics_actuator::JointPositions msg;
	
	// open gripper
	msg = createGripperPositionCommand(0.011);
	gripperPublisher.publish(msg);
}
void closeGripper() {
	brics_actuator::JointPositions msg;

	// close gripper
	msg = createGripperPositionCommand(0);
	gripperPublisher.publish(msg);
}

void initializeArm() {
	// move arm back close to calibration position
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 1.44;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);
}


// move arm once up and down
void moveArm() {
	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);

	// move arm straight up. values were determined empirically
	jointvalues[0] = 2.95;
	jointvalues[1] = 1.05;
	jointvalues[2] = -2.44;
	jointvalues[3] = 1.73;
	jointvalues[4] = 2.87;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(5).sleep();

	// move arm back close to calibration position
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.11;
	jointvalues[3] = 0.11;
	jointvalues[4] = 0.12;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(2).sleep();
}

void waveArm() {
	// arm joint 4 moves out
	moveKukaArmJoints(0.11, 0.11, -0.11, deg2rad(60), 1.44);
	ros::Duration(2).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.11, 0.11, 1.44);
	//ros::Duration(1).sleep();
	//moveKukaArmJoints(0.11, 0.11, -0.11, deg2rad(60), 1.44);
	//ros::Duration(1).sleep();
	//moveKukaArmJoints(0.11, 0.11, -0.11, 0.11, 1.44);
}

void idleBaseShimmy() {
	moveKukaArmJoints(0.36, 0.11, -0.11, 0.11, 1.44);
	ros::Duration(1).sleep();
	initializeArm();
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.36, 0.11, -0.11, 0.11, 1.44);
	ros::Duration(1).sleep();
	initializeArm();
	ros::Duration(1).sleep();
}

void idleBehaviorGripper() {
	openGripper();
	ros::Duration(2).sleep();
	closeGripper();
	ros::Duration(2).sleep();
	openGripper();
	ros::Duration(2).sleep();
}

void idleBehaviorTopJoint(bool jointPos) {
	double jointvalue4;

	if (jointPos) {
		jointvalue4 = 1.44;
	} else { 
		jointvalue4 = 4.32; 
	}
	moveKukaArmJoints(0.11, 0.11, -0.11, 0.11, jointvalue4);

	//ros::Duration(5).sleep();

}

void gripAndTwist() {
	openGripper();
	ros::Duration(2).sleep();
	closeGripper();
	ros::Duration(2).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.11, deg2rad(60), 0.12);
	moveKukaArmJoints(0.11, 0.11, -0.11, deg2rad(60), 2.0);
	initializeArm();
}

void scratchEquiv() {
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(7), 1.44);
	closeGripper();
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(30), 1.44);
	openGripper();
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(7), 1.44);
	ros::Duration(1).sleep();
	closeGripper();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(30), 1.44);
	ros::Duration(1).sleep();
	initializeArm();
}

void actionWhileSpeechOne() {
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(7), 1.44);
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(30), 1.44);
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(7), 1.44);
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, -0.40, deg2rad(30), 1.44);
	ros::Duration(1).sleep();
	initializeArm();
}

void actionWhileSpeechTwo() {
	
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.30, -0.40, deg2rad(7), 1.60);
	ros::Duration(3).sleep();
	initializeArm();
	ros::Duration(2).sleep();
}

void waveInFace() {

	brics_actuator::JointPositions msg;
	std::vector<double> jointvalues(5);
	
	jointvalues[0] = 0.11;
	jointvalues[1] = 0.11;
	jointvalues[2] = -0.12;
	jointvalues[3] = 0.11;
	jointvalues[4] = 1.44;
	msg = createArmPositionCommand(jointvalues);
	armPublisher.publish(msg);

	ros::Duration(1).sleep();
	

	moveKukaArmJoints(0.11, 0.11, deg2rad(-140), 0.70, 1.44);
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, deg2rad(-80), 0.70, 1.44);
	ros::Duration(2).sleep();

	moveKukaArmJoints(0.11, 0.11, deg2rad(-140), 0.70, 1.44);
	ros::Duration(1).sleep();
	moveKukaArmJoints(0.11, 0.11, deg2rad(-80), 0.70, 1.44);
	ros::Duration(1).sleep();
	initializeArm();
}

void moveUpDistract() {
	// stand up joints 3 and 4
	moveKukaArmJoints(0.11, 0.11, deg2rad(-100), deg2rad(102), 0.12);
	ros::Duration(2).sleep();
	
	// rotate base
	moveKukaArmJoints(deg2rad(84), 0.11, deg2rad(-100), deg2rad(102), 0.12);
	ros::Duration(2).sleep();

	standUpWaveHelper();
	ros::Duration(1).sleep();
	// spinny base
	moveKukaArmJoints(deg2rad(200), 0.11, deg2rad(-100), deg2rad(102), 0.12);
	ros::Duration(1).sleep();
	moveKukaArmJoints(deg2rad(200), 0.11, deg2rad(-100), deg2rad(190), 0.12);
	ros::Duration(1).sleep();
	moveKukaArmJoints(deg2rad(200), 0.11, deg2rad(-100), deg2rad(102), 0.12);
	ros::Duration(2).sleep();
	initializeArm();
}

void pointOut() {
	ros::Duration(2).sleep();
	// move up first
	moveKukaArmJoints(3, deg2rad(60), deg2rad(-130), deg2rad(102), 0.12);
	//moveKukaArmJoints(deg2rad(70), 0.11, deg2rad(-20), deg2rad(102), 0.12);
	ros::Duration(3).sleep();
	moveKukaArmJoints(3, deg2rad(60), deg2rad(-130), deg2rad(190), 0.12);
	ros::Duration(8).sleep();
	initializeArm();	
}

void idleBehavior(const ros::TimerEvent& event) {
	int randomBehavior = rand();
	
	if (randomBehavior % 4 == 0) {
		idleBehaviorGripper();
		ROS_INFO("idle grip");
	} else if (randomBehavior% 4 == 1) {
		jointval4 = !jointval4;
		idleBehaviorTopJoint(jointval4);
		ROS_INFO("idle top joint");
	} else if (randomBehavior % 4 == 2) {
		waveArm();
		ROS_INFO("small wave");
	} else {
		idleBaseShimmy();
		ROS_INFO("base shimmy");	
	}
}

void gameCallback(const youbot_ros_hello_world::GameState::ConstPtr& msg) {
	int highScore = msg->highScore;	
	int roundsComplete = msg->roundsComplete;
	int losses = msg->losses;
	bool gameInProgress = msg->gameStart;
	ROS_INFO("high score: %d", highScore);
	ROS_INFO("rounds complete: %d", roundsComplete);
	ROS_INFO("losses: %d", losses);

	if (!gameInProgress) {
		waveArm();
		actionWhileSpeechOne();
	}
	else if (instructions && gameInProgress) {
		waveArm();
		instructions = false;
	}

	
}

void timeCallback(const youbot_ros_hello_world::TimeState::ConstPtr& timeMsg) {
	int timeLeft = (int) round(timeMsg->minutesLeft / 60);
	ROS_INFO("Time update: %d minutes remaining", timeLeft);
	if (timeLeft == 1) {
		//actionWhileSpeechOne();
	} else if (timeLeft == 6) {
		// behavior 1, scratch head
		scratchEquiv();
	} else if (timeLeft == 5) {
		// behavior 2, waving
		waveInFace();
	} else if (timeLeft == 3) {
		// behavior 3, ask for expo marker
		pointOut();
	} else if (timeLeft == 2) {
		// behavior 4, bopping
		moveUpDistract();
	}
}

void gameUpdateCallback(const youbot_ros_hello_world::GameUpdate::ConstPtr& gameMsg) {
	int rounds = gameMsg->rounds;
	// add voice lines to unity game, all unity game code
	idleBehaviorGripper();
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "youbot_ros_hello_world");
	ros::init(argc, argv, "listener");
	ros::NodeHandle n;
	bool gameEnd = false;

	ros::MultiThreadedSpinner spinner(4);

	armPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/arm_controller/position_command", 10);
	gripperPublisher = n.advertise<brics_actuator::JointPositions>("arm_1/gripper_controller/position_command", 10);

	gameStateSubscriber = n.subscribe("game_state", 1000, gameCallback);
	timeStateSubscriber = n.subscribe("time_state", 1000, timeCallback);
	gameUpdateSubscriber = n.subscribe("game_update", 1000, gameUpdateCallback);
	ros::Timer timer = n.createTimer(ros::Duration(20), idleBehavior);


	spinner.spin();
	
	ros::Duration(1).sleep();

	// ros::shutdown();

	return 0;
}

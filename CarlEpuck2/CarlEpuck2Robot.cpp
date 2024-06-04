
#include <iostream>

//Linux Pi5
#include<thread>

using namespace std;

#include "CarlEpuck2.h"
#include "CarlEpuck2Robot.h"

using namespace yarp::carl;

#include <yarp/os/all.h>

using namespace yarp::os;

#include <CarlCommThread.h>


#define myTrace   if(m_log <= Log::TraceType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace
#define myDebug   if(m_log <= Log::DebugType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug
#define myInfo    if(m_log <= Log::InfoType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info
#define myWarning if(m_log <= Log::WarningType) yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning
#define myError   if(m_log <= Log::ErrorType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error
#define myFatal   if(m_log <= Log::FatalType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).fatal

//#define ifLogTrace   if(m_log <= yarp::os::Log::TraceType)	yarp::os::Log().trace()
#define ifLogDebug   if(m_log <= yarp::os::Log::DebugType)	yarp::os::Log().debug()	
//#define ifLogInfo    if(m_log <= yarp::os::Log::InfoType)	yarp::os::Log().info()	


CarlEpuck2Behavior::CarlEpuck2Behavior(CarlEpuck2Robot * robot, yarp::carl::CarlEpuck2 * device) : 
	m_robot(robot), m_device(device) {   
};

void CarlEpuck2Behavior::run() {
}

bool CarlEpuck2Behavior::threadInit() {
	return true; 
}

void CarlEpuck2Behavior::threadRelease() {
	
}

// Braitenberg
class BraitenbergBehavior : public CarlEpuck2Behavior
{
public:
public:
	BraitenbergBehavior(yarp::carl::CarlEpuck2Robot* robot = nullptr, yarp::carl::CarlEpuck2* device = nullptr):  
		CarlEpuck2Behavior(robot, device)  {};

	virtual void run() {

		while (running) {

		}
	};


	virtual bool threadInit() {
		return true; 
	}

};



class BodyBehavior : public CarlEpuck2Behavior
{
public:

	int steps;
public:
	BodyBehavior(yarp::carl::CarlEpuck2Robot* robot, yarp::carl::CarlEpuck2* device, int steps = 0) :CarlEpuck2Behavior(robot, device), steps(steps) {};

	virtual void run() {


		int dt = 0;

		for (int i = 0; !this->isStopping() && (steps == 0 || i < steps) && dt != -1; i++) {

			m_robot->getSensorInput();
			m_device->transmitSensorInput();  // yarp writing port   values --> component of EPuck 

			m_robot->blinkLeds();

			m_robot->setActuators();
			dt = m_robot->step(1);

			this->yield(); 
			std::this_thread::sleep_for(2 * 64ms);
		};
	};

};


const double CarlEpuck2Robot::MAX_SPEED = 6.28;

const char* CarlEpuck2Robot::proximitySensorNames[] = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };

const char* CarlEpuck2Robot::lightSensorNames[] = { "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7" };

const char* CarlEpuck2Robot::ledNames[] = { "led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9" };


const double CarlEpuck2Robot::weights[proximitySensorCount][2] = {   // Braitenberg coefficients  controller e-puck_avoid_obstacles.c
	{-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
	{0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0} };


const double CarlEpuck2Robot::offsets[2] = { 0.5 * MAX_SPEED, 0.5 * MAX_SPEED };

CarlEpuck2Robot::CarlEpuck2Robot(CarlEpuck2* device): device(device) {

	robot = nullptr;

	behavior = nullptr;

	// init values 
	resetActuatorValues();
}



double CarlEpuck2Robot::getTime() { 
	return 0;  
}

void CarlEpuck2Robot::runBody(int steps) {

	behavior = new BodyBehavior(this, device, steps);    // how will free this ???

	active = std::chrono::steady_clock::now(); 

	behavior->start(); 
	
}


void CarlEpuck2Robot::halt() {

	if (behavior) {

		behavior->stop(); 

		delete behavior;
		behavior = nullptr;
	}

}



void CarlEpuck2Robot::setSpeed(double left, double right) {


	std::lock_guard<std::mutex> guard(mtx); 

	speeds[LEFT] = left; 
	speeds[RIGHT] = right; 

}


void CarlEpuck2Robot::setLedValues(const std::vector<int> indeces, const int led[]) {

	std::lock_guard<std::mutex> guard(mtx);

	for(auto iter = indeces.begin(); iter < indeces.end(); iter++) {
		ledValues[*iter] = led[*iter];
	}
}


void CarlEpuck2Robot::setText(std::string text) {

	std::lock_guard<std::mutex> guard(mtx);

	this->text = text;
}


void CarlEpuck2Robot::initDevices() {

	// Actuators


	robot->stopMotors(); 


}


void CarlEpuck2Robot::resetActuatorValues() {
	for (int i = 0; i < 2; i++)
		speeds[i] = 0.0;
	for (int i = 0; i < ledCount; i++)
		ledValues[i] = false;
}

void CarlEpuck2Robot::getSensorInput() {

	// Streaming with controler time step (scalar or 1-dim vectors) 

	for (int i = 0; i < proximitySensorCount; i++) {
		psValues[i] = robot->getIr(i); // trunc

	}

		tofValue = robot->getDistance(); // trunc
	
	whlsValues[LEFT] = robot->getLeftSteps();
	whlsValues[RIGHT] = robot->getRightSteps();


}


bool CarlEpuck2Robot::cliffDetected() {

	return false;
}

void CarlEpuck2Robot::setActuators() {

	robot->go(speeds[LEFT]*100, speeds[RIGHT]*100);

}


void CarlEpuck2Robot::blinkLeds() {
	static int counter = 0;
	counter++;

	if (counter % 20 < 2) {  // 2 = blink length
		ledValues[3] = 0x00FF00;  // RGB 
		ledValues[5] = 0xFF0000;  // RGB 
	}

	if (robot->getDistance() < 200)
			ledValues[0] = true;  // top front 
}

void CarlEpuck2Robot::calculateBraitenberg() {  //coeficients
	for (int i = 0; i < 2; i++) {
		speeds[i] = 0.0;
		for (int j = 0; j < proximitySensorCount; j++) 
			speeds[i] += proximitySensorValues[j] * weights[j][i];

		speeds[i] = offsets[i] + speeds[i] * MAX_SPEED; 
		if (speeds[i] > MAX_SPEED)
			speeds[i] = MAX_SPEED;
		else if (speeds[i] < -MAX_SPEED)
			speeds[i] = -MAX_SPEED;
	
	}
}

void CarlEpuck2Robot::runBraitenberg(int steps) {

	int turned = 0;
	int dt = 0; 

	for (int i = 0; i < steps && dt != -1; i++) { 


		resetActuatorValues();

		getSensorInput();
		device->transmitSensorInput();  // yarp writing port   values --> component of EPuck 

		blinkLeds();
		if (cliffDetected()) {
			goBackwards();
			if (turned++ % 4)
				turnLeft();
			else
				turnRight(); 
		}
		else {
			calculateBraitenberg();
		}
		setActuators();
		dt = step(1);

	};
}


void CarlEpuck2Robot::goBackwards() {
}

void CarlEpuck2Robot::turnLeft() {
}

void CarlEpuck2Robot::turnRight() {
}


int CarlEpuck2Robot::step(int steps) {
	int dt = 0;  // -1 exit, 0 expected, > 0 delay, ..> see Robot, sync
	return dt;
}

void CarlEpuck2Robot::passiveWait(double sec) {
}


bool CarlEpuck2Robot::init() {

	auto name = "e-puck2";
	if (name == "e-puck2") {
		printf("e-puck2 robot\n");
		wbTimeStep = 64;
		wbCameraTimeStep = 64;
	}

	wbBasicTimeStep = 16;

	resetActuatorValues();   

	initDevices();


	int ok = step();

  return true;
}



void CarlEpuck2Robot::release() {

	if (robot)
		delete robot;


}

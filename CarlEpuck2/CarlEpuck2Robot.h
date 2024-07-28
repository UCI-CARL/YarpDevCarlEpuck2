
#pragma once	// solves order related issues

#include <yarp/os/Thread.h>
#include <yarp/os/ConstString.h>
using yarp::os::ConstString;
#include <yarp/os/Log.h>

#include <carl/boost_ext.hpp>

#include <stdio.h>
#include <vector>
#include <chrono>

class CarlCommThread; // robot

class CarlEpuck2Behavior;
class BraitenbergBehavior;
class BodyBehavior;

namespace yarp {
	namespace carl {
		class CarlEpuck2;  // yarp device
		class CarlEpuck2Robot;
	}
}

class CarlEpuck2Behavior : public yarp::os::Thread   // PeriodicThread  not neccessary when using step(ms) 
{
public:
	yarp::carl::CarlEpuck2Robot* m_robot;
	yarp::carl::CarlEpuck2* m_device;
public:
	CarlEpuck2Behavior(yarp::carl::CarlEpuck2Robot* robot = nullptr, yarp::carl::CarlEpuck2* device = nullptr);
	virtual void run();
	virtual bool threadInit();
	virtual void threadRelease();
protected:
	bool running; 
};

class yarp::carl::CarlEpuck2Robot
{
private:


public:
	CarlEpuck2Robot(CarlEpuck2* device);

	bool init();
	void release();

	virtual int step(int steps = 1);

	virtual void runBraitenberg(int steps = 1);
	virtual void runBody(int steps = 0);   

	virtual void halt(); // stop current behavior

	// thread safe
	virtual void setSpeed(double left, double right); 
	virtual void setLedValues(std::vector<int> indeces, const int led[]);
	virtual void setText(std::string text);

	static const int LEFT = 0;
	static const int RIGHT = 1;
	static const double MAX_SPEED; 

	friend CarlEpuck2;
	friend CarlEpuck2Behavior;
	friend BraitenbergBehavior;
	friend BodyBehavior;

	// do sleep, do wakeup, do explore [maze| [open]field]]  
	// state, mode, behavior; 
	enum Behaviro {IDLE, AWAKE, WALL_FOLLOWING, EXPLORE_MAZE, EXPLORE_OPEN_FIELD, EXPLOIT, SLEEP};

	double getTime();

protected:  

	virtual void initDevices(); 

	virtual void blinkLeds();
	virtual void setActuators();
	virtual void getSensorInput();
	virtual void getCamInput();
	virtual void resetActuatorValues();

	virtual void goBackwards();
	virtual void turnLeft(); 
	virtual void turnRight();

	virtual void passiveWait(double sec);
	virtual void calculateBraitenberg();

	virtual bool cliffDetected(); 

//private:
public:
	// E-Puck 2
	// https://www.gctronic.com/doc/index.php/e-puck2#Programmer

	CarlCommThread* robot;    // see also  e-puck_wifi  

//private:
public:
	// Sensors
	//webots::Accelerometer* accelerometer;
	double accelValues[3];

	//webots::Gyro* gyro;
	double gyroValues[3];

	// Wheels 41 mm
	//webots::PositionSensor* leftWheel;
	//webots::PositionSensor* rightWheel;
	int whlsValues[2];

	// Proximity: up to 6 cm 
	const static int proximitySensorCount = 8;
	//webots::DistanceSensor* proximitySensors[proximitySensorCount];  // 6 cm
	static const char* proximitySensorNames[proximitySensorCount]; // = { "ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7" };
	double proximitySensorValues[proximitySensorCount];
	int psValues[proximitySensorCount]; // raw values 

	static const double weights[proximitySensorCount][2]; // = { {-1.3, -1.0}, {-1.3, -1.0}, {-0.5, 0.5}, {0.0, 0.0},
												// {0.0, 0.0},   {0.05, -0.5}, {-0.75, 0},  {-0.75, 0} };
	static const double offsets[2]; // = { 0.5 * MAX_SPEED, 0.5 * MAX_SPEED };

	// Distance: up to 2 m
	//webots::DistanceSensor* tof; // Time of F(l)ight
	int tofValue; 

	const static int lightSensorCount = 8;
	//webots::LightSensor* lightSensors[lightSensorCount];
	static const char* lightSensorNames[lightSensorCount]; //= { "ls0", "ls1", "ls2", "ls3", "ls4", "ls5", "ls6", "ls7" };
	double lightSensorValues[lightSensorCount];
	int lsValues[lightSensorCount]; // raw values


	static const int CAM_WIDTH = 160; // QQVGA
	static const int CAM_HEIGHT = 120;
	static const int CAM_BYTES = CAM_WIDTH * CAM_HEIGHT * 3;   // issue int
	int temp_w;
	int temp_h;
	//webots::Camera* camera;  // 160 x 120
	int camValues[CAM_BYTES];
	//unsigned char* img;  // ??
	//unsigned char img[CAM_WIDTH * CAM_HEIGHT * 2];     // QQVGA, RGB565
	unsigned char img[CAM_WIDTH * CAM_HEIGHT * 4];     // bga


	// Actuators

	double speeds[2];

	// LEDs
	
	// C:\Program Files\Webots\projects\robots\gctronic\e-puck\controllers\e-puck_avoid_obstacles\e-puck_avoid_obstacles.c
	const static int ledCount = 10;
	//webots::LED* leds[ledCount]; 
	static const char* ledNames[ledCount]; // = { "led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9" };
	int ledValues[ledCount];


	// Speaker 
	//webots::Speaker* speaker;
	std::string text;

	int wbTimeStep;
	int wbCameraTimeStep;
	int wbBasicTimeStep; 

	std::chrono::time_point<std::chrono::steady_clock> active;  
	
	CarlEpuck2Behavior* behavior;  // Default Idle

	CarlEpuck2* device;    

	std::mutex mtx;
};

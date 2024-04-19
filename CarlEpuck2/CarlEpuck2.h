// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#pragma once	// solves order related issues

#include <yarp/dev/DeviceDriver.h>
#include <yarp/conf/system.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Value.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Portable.h>
#include <yarp/os/ConnectionWriter.h>
#include <yarp/os/ConstString.h>
using yarp::os::ConstString;

#include <yarp/os/NetInt32.h>
#include <yarp/os/RpcServer.h>
#include <yarp/os/Log.h>

#include <carl/boost_ext.hpp>

#include <stdio.h>

namespace yarp {
	namespace carl {
		class CarlEpuck2;
		class CarlEpuck2Robot;
	}
}

using namespace yarp::os;
class yarp::os::Bottle;
class yarp::os::RpcServer;

using namespace yarp::carl;

// RPC might be better still
class CarlWbSupervisorThread : public yarp::os::Thread
{
public:
	yarp::carl::CarlEpuck2* m_supervisor;
	yarp::os::Log::LogType m_log;
public:
	CarlWbSupervisorThread(yarp::carl::CarlEpuck2* supervisor);
	CarlWbSupervisorThread() : m_supervisor(NULL) { }
	virtual void run();
	virtual bool threadInit();
	virtual void threadRelease();
protected:
	void loopback();
};


class CarlCommThread;

// time stamp,   other scructured data
class EPuck2Sensors : public Portable {
public:
	EPuck2Sensors();

	void initialize();
	bool write(ConnectionWriter& connection) const override;
	bool read(ConnectionReader& connection) override;

	static const int N = 8;
	unsigned long long t;  // ms
	int tof;
	int ps[N];
	int ls[N];
	int whls[2];

	int accl[3];  // x  y z  m/s^2 
	int gyro[3];  // gX GY GZ   // Neigung => Cost 

	// battery -> energy consumptions ??
	// odometer

	bool b_tof, b_ps, b_ls, b_whls, b_accl, b_gyro;
};

// time stamp,   other scructured data
class EPuck2Actuators: public Portable {
public:
	EPuck2Actuators();

	void initialize();
	bool write(ConnectionWriter& connection) const override;
	bool read(ConnectionReader& connection) override;

	static const int N = 2;
	static const int LEFT = 0;  
	static const int RIGHT = 1;
	static const int ledCount = 10;

	double speeds[N];  // left, right
	int led[ledCount];
	std::string spk;

	bool b_vel, b_red, b_rgb, b_frnt, b_chss, b_spk; 
};

class EPuck2ActuatorsReceiver: public BufferedPort<EPuck2Actuators>
{
public:
	yarp::carl::CarlEpuck2* puck;

	using BufferedPort<EPuck2Actuators>::onRead;
	void onRead(EPuck2Actuators& b) override;
};

class yarp::carl::CarlEpuck2 : public yarp::dev::DeviceDriver
{
private:

	yarp::os::RpcServer m_port_supervisor;	// load, start, stop, robot_position for re-enforce learning, maybe sync

	// TCP socket 
	CarlCommThread *m_comm; 

	CarlWbSupervisorThread *m_supervisor;  // rpc 
	int m_loopback;

	double m_speed;
	double m_min_sleep, m_max_sleep, m_sum_sleep;
	unsigned long long m_sleeps;

	yarp::os::Log::LogType m_log;
 
	yarp::os::BufferedPort<EPuck2Sensors> m_port_sensors; // multitype message 

	EPuck2ActuatorsReceiver m_port_actuators;  // multi type messages: (timestamp, motor, led, ...)

	CarlEpuck2Robot* m_robot;  // wrapper with behavior with weak yarp dependencies

	std::FILE *m_steps;  // csv with wheel steps: ms;left;right  (1000 steps per revolution)  
	std::FILE *m_trajectory;  // csv trajectory: ms, x, y, tau  (in m and rad)

public:
	CarlEpuck2();

	// Device Driver IF
	virtual bool close();
	virtual bool open(yarp::os::Searchable& config);

	virtual void transmitSensorInput();  // yarp write 
	virtual bool receiveActuatorOutput(); 

	friend CarlCommThread;
	friend CarlWbSupervisorThread;
	friend EPuck2ActuatorsReceiver; 

protected:

private:

};


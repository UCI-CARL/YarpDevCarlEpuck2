
#include <iostream>
using namespace std;

#include "CarlEpuck2.h"
#include "CarlEpuck2Robot.h"
#include "CarlCommThread.h"

using namespace yarp::dev;

#include <yarp/os/all.h>

using namespace yarp::os;
using namespace yarp::dev;

#define myTrace   if(m_log <= Log::TraceType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace
#define myDebug   if(m_log <= Log::DebugType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug
#define myInfo    if(m_log <= Log::InfoType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info
#define myWarning if(m_log <= Log::WarningType) yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning
#define myError   if(m_log <= Log::ErrorType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error
#define myFatal   if(m_log <= Log::FatalType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).fatal

#define ifLogDebug   if(m_log <= yarp::os::Log::DebugType)	yarp::os::Log().debug()	

#include <yarp/os/impl/PlatformTime.h>

// patch YARP deprecated mixup
#ifndef BOTTLE_TAG_VOCAB32
  #define BOTTLE_TAG_VOCAB32 BOTTLE_TAG_VOCAB
#endif

#include "EPuckCommandPacket.hpp"
#include "Communication.hpp"
#include "Wrapper.hpp"

#include <webots/remote_control.h>
#include <webots/nodes.h>

CarlWbSupervisorThread::CarlWbSupervisorThread(yarp::carl::CarlEpuck2 * supervisor) : m_supervisor(supervisor)
{
	m_log = supervisor->m_log;
}

void CarlWbSupervisorThread::loopback() {
	Bottle cmd;
	Bottle response;
	m_supervisor->m_port_supervisor.read(cmd, true);
	ACE_Time_Value sleep_period;
	sleep_period.set_msec(50); // 50 ms

	ACE_OS::sleep(sleep_period);
	response.addString("you");
	response.addString("said");
	response.append(cmd);

	m_supervisor->m_port_supervisor.reply(response);
}

/*
	yarp rpc /wb/e-puck2/supervisor
	CAUTION: a supervisor requires an (external) robot was loaded before, which in turn requires a World file.
*/
void CarlWbSupervisorThread::run() {
	if (m_supervisor->m_loopback > 0) {
		for (int loop = 1; loop <= m_supervisor->m_loopback; loop++)
			loopback();
		return;
	}

	while (true) {

		Bottle rpcRequest, rpcResponse;
		m_supervisor->m_port_supervisor.read(rpcRequest, true);

		std::string command = rpcRequest.get(0).asString(); 

		if (command == "range") // arg1 sensor  ps tof
		{
			auto sensor = rpcRequest.get(1).asString();
		
			if (sensor == "ps") {
				rpcResponse.addInt(60);
				rpcResponse.addInt(250);
			}
			else if (sensor == "tof") {
				rpcResponse.addInt(30);  // this should be the min
				rpcResponse.addInt(80);  // this defines show close, the less the closer, 
			}

		}
		else
		if (command == "quit") {
			rpcResponse.addString("bye");
		} 
		else 
		if (command == "init") 
		{
			m_supervisor->m_robot->init();
			rpcResponse.addString("E-Puck2 ready");
		} 
		else
		if (command == "release")
		{
			m_supervisor->m_robot->release();
			rpcResponse.addString("E-Puck2 released");
		} 
		else
		if (command == "step")
		{
			int steps = 1;
			if (rpcRequest.size() > 1) // read optional parameter
				steps = rpcRequest.get(1).asInt();
			int dt = m_supervisor->m_robot->step(steps);
			rpcResponse.addInt(dt);
		} 
		else
		if (command == "braitenberg")
		{
			int steps = 1;
			if (rpcRequest.size() > 1) // read optional parameter
				steps = rpcRequest.get(1).asInt();
		    m_supervisor->m_robot->runBraitenberg(steps);
			rpcResponse.addInt(0);
		} 
		else
		if (command == "body") 
		{
			m_supervisor->m_robot->runBody();			
		}
		else
		if (command == "halt")
		{
			m_supervisor->m_robot->halt();

		}

		m_supervisor->m_port_supervisor.reply(rpcResponse);

		if (command == "quit") {
			m_supervisor->close();  // quit the device
			exit(0); 
			break;
		}

	}
}


bool CarlWbSupervisorThread::threadInit() {
	return true;
}

void CarlWbSupervisorThread::threadRelease() {
}


EPuck2Sensors::EPuck2Sensors() {
	printf("EPuck2Sensors() \n");
	initialize();
};

void EPuck2Sensors::initialize()
{
	b_tof = b_ps = b_ls = b_whls = b_accl = b_gyro = false;
	t = 0; // sim time in ms
	tof = 0;
	for (int i = 0; i < N; i++) ps[i] = 0;
	for (int i = 0; i < N; i++) ls[i] = 0;
	for (int i = 0; i < 2; i++) whls[i] = 0;	
	for (int i = 0; i < 3; i++) accl[i] = 0;
	for (int i = 0; i < 3; i++) gyro[i] = 0;
}


bool EPuck2Sensors::write(ConnectionWriter& connection) const 
{
	connection.appendInt32(BOTTLE_TAG_LIST);
	int ct = 2 + (b_tof ? 1 + 1: 0) + (b_ps ? 1 + N : 0) + (b_ls ? 1 + N : 0)
		   + (b_whls ? 1 + 2 : 0) + (b_accl ? 1 + 3 : 0) + (b_gyro ? 1 + 3 : 0);
	connection.appendInt32(ct);

	int e = 0; 

	// always send a time stamp (ms)
	connection.appendInt32(BOTTLE_TAG_VOCAB32);
	connection.appendInt32(Vocab::encode("t"));
	e++;
	connection.appendInt32(BOTTLE_TAG_INT64);// 
	connection.appendInt64(t);
	e++;

	if (b_tof) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("tof"));
		e++;
		connection.appendInt32(BOTTLE_TAG_INT32);// 
		connection.appendInt32(tof);
		e++;
	}
	if (b_ps) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("ps"));
		e++;
		for (int i = 0; i < N; i++) {
			connection.appendInt32(BOTTLE_TAG_INT32); 
			connection.appendInt32(ps[i]);
			e++;
		}
	}
	if (b_ls) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("ls"));
		e++;
		for (int i = 0; i < N; i++) {
			connection.appendInt32(BOTTLE_TAG_INT32); 
			connection.appendInt32(ls[i]);
			e++;
		}
	}


	// Wheels whls
	if (b_whls) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("whls"));
		e++;
		for (int i = 0; i < 2; i++) {
			connection.appendInt32(BOTTLE_TAG_INT32);
			connection.appendInt32(whls[i]);
			e++;
		}
	}

	// Gyro
	if (b_gyro) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("gyro"));
		e++;
		for (int i = 0; i < 3; i++) {
			connection.appendInt32(BOTTLE_TAG_INT32);
			connection.appendInt32(gyro[i]);
			e++;
		}
	}


	// Accelromenter aclm
	if (b_accl) {
		connection.appendInt32(BOTTLE_TAG_VOCAB32);
		connection.appendInt32(Vocab::encode("accl"));
		e++;
		for (int i = 0; i < 3; i++) {
			connection.appendInt32(BOTTLE_TAG_INT32);
			connection.appendInt32(accl[i]);
			e++;
		}
	}

	assert(e == ct);

	connection.convertTextMode(); // if connection is text-mode, convert!
	return true;
}

bool EPuck2Sensors::read(ConnectionReader& connection) 
{
	return true;
}


EPuck2Actuators::EPuck2Actuators()  {
	printf("EPuck2Actuators() \n");
	initialize(); 
};

void EPuck2Actuators::initialize() 
{
	b_vel = b_red = b_rgb = b_frnt = b_chss = b_spk = false;
	for (int i = 0; i < N; i++) speeds[i] = .0;
	for (int i = 0; i < ledCount; i++) led[i] = 0;
	spk = "";
}

bool EPuck2Actuators::write(ConnectionWriter& connection) const
{
	// not applicable
	return true;
}

bool EPuck2Actuators::read(ConnectionReader& connection) 
{
	initialize();  // Object is cached therefore the constructor not called every time

	connection.convertTextMode(); // if connection is text-mode, convert!


	int tag = connection.expectInt32();
	// header which actuators the message contains
	if (tag != BOTTLE_TAG_LIST) {   //  [vel] 0.2 0.3  [red] 0 1 0 1    [acc]  [pos]    [rgb]
		printf("skipped, invalid tag: %d\n", tag);
		return false;
	}

	// [vel] 0.1 0.2
	int ct = connection.expectInt32();  // elements 
	if (ct < 1) {
		printf("skipping empty message  %d\n", ct);
		return false;
	}


	for (int e = 0; e < ct; ) {

		tag = connection.expectInt32();

		if (tag != BOTTLE_TAG_VOCAB32) {
			printf("vocab expected: %d\n", tag);
			return false;
		}

		auto code = connection.expectInt32();
		e++;

		auto s = yarp::os::Vocab::decode(code);   // Vocab32  YARP 3.5

		if (code == Vocab::encode("vel")) {
			for (int i = 0; i < 2; i++) {     
				tag = connection.expectInt32();
				assert(tag == BOTTLE_TAG_FLOAT64);
				double value = connection.expectFloat64();
				e++;
				if (value < -7.5 || value > 7.5) {   
					printf("motor velocity out of range:  %f\n", value);
					return false;
				}
				speeds[i] = value;
			}
			b_vel = true; 
		} else
		if (code == Vocab::encode("red")) {
			for (int i = 0; i < 4; i++) {    
				tag = connection.expectInt32();
				assert(tag == BOTTLE_TAG_INT32);
				auto value = connection.expectInt32();
				e++;
				if (value != 0 && value != 1) {
					printf("red led %d out of range:  %d\n", i, value);
					return false;
				}
				led[i * 2] = value;
			}
			b_red = true; 
		} else
		if (code == Vocab::encode("rgb")) {
			for (int i = 0; i < 4; i++) {    
				tag = connection.expectInt32();
				assert(tag == BOTTLE_TAG_INT32);
				auto value = connection.expectInt32();
				e++;
				led[i * 2 + 1] = value;
			}
			b_rgb = true; 
		} else
		if (code == Vocab::encode("frnt")) {
			tag = connection.expectInt32();
			assert(tag == BOTTLE_TAG_INT32);
			auto value = connection.expectInt32();
			e++;
			if (value != 0 && value != 1) {
				printf("front led out of range:  %d\n", value);
				return false;
			}
			led[9] = value;
			b_frnt = true; 
		} else
		if (code == Vocab::encode("chss")) {
			tag = connection.expectInt32();
			assert(tag == BOTTLE_TAG_INT32);
			auto value = connection.expectInt32();
			e++;
			if (value != 0 && value != 1) {
				printf("chassi led out of range:  %d\n", value);
				return false;
			}
			led[8] = value;
			b_chss = true;
		} else
		if (code == Vocab::encode("spk")) {
			tag = connection.expectInt32();
			assert(tag == BOTTLE_TAG_STRING);
			auto value = connection.expectString();
			e++;
			spk = value;
			b_spk = true;
		}
	}

	return true;

}

CarlEpuck2::CarlEpuck2() {

	m_robot = new CarlEpuck2Robot(this);   


}



bool CarlEpuck2::open(yarp::os::Searchable& config) {


	ConstString log = config.check("log", Value("info"), "Verbosity: error, warning, info, debug, trace (see Log.h for details)").asString();

	if (log == "trace") m_log = Log::TraceType; // 0
	else if (log == "debug") m_log = Log::DebugType; // 1
	else if (log == "info") m_log = Log::InfoType;	 // 2
	else if (log == "warning") m_log = Log::WarningType; // 3
	else if (log == "error") m_log = Log::ErrorType; // 4
	else if (log == "fatal") m_log = Log::FatalType; // 5
	else; // leave the default

	myInfo("CarlEpuck2::open");
	myInfo("Log: %s (%d)", log.c_str(), m_log);


	//// open logging files
	m_steps = fopen("ncpuck2_steps.csv", "w");
	fprintf(m_steps, "ms; steps_l; steps_r\n");

	m_trajectory = fopen("ncpuck2_trajectory.csv", "w");
	fprintf(m_trajectory, "ms; x; y; tau; dsteps_l; dsteps_r; v_l; v_r; v; w; dx; dy; dtau\n");


	// moved into robot, see below
	m_comm = new CarlCommThread(this);
	m_comm->init();
	m_comm->start();

	m_robot->robot = m_comm;

	ConstString supervisor_port_name = config.check("port", Value("/e-puck2/supervisor"), "RPC Port for E-Puck").asString();
	myInfo("Webots Supervisor port: %s", supervisor_port_name.c_str());

	// Testing
	m_loopback = config.check("loopback", Value(0), "RPC Loopback Testing").asInt();
	myDebug("loopback: %d", m_loopback);

	m_port_supervisor.open(supervisor_port_name);
	myDebug("Webots Supervisor port opened.");

	m_supervisor = new CarlWbSupervisorThread(this);
	m_supervisor->start();

	ConstString sensors_port_name = config.check("port", Value("/e-puck2/sensors"), "Writing Port for Sensor").asString();
	myInfo("port: %s", sensors_port_name.c_str());
	m_port_sensors.open(sensors_port_name);
	myDebug("Webots Sensors port opened.");

	ConstString actuators_port_name = config.check("port", Value("/e-puck2/actuators"), "Reading Port for Auctors").asString();
	myInfo("port: %s", actuators_port_name.c_str());

	m_port_actuators.useCallback();  // input should go to onRead() callback
	m_port_actuators.open(actuators_port_name);
	m_port_actuators.puck = this; 

	myDebug("E-Puck2 Actuators port opened.");

	m_robot->init();

	m_robot->runBody();

  return true;
}

bool CarlEpuck2::close() {
	
	m_robot->robot->stopMotors();

	m_port_supervisor.close();
	myDebug("Webot supervisor port closed.");

	if (m_robot)
		delete m_robot;

	if (m_steps) 
		fclose(m_steps);

	if (m_trajectory) 
		fclose(m_trajectory); 
	
	myInfo("CarlEpuck2 device closed.");
	
	return true;
}


#include <ctime>  
#include <cmath>

void CarlEpuck2::transmitSensorInput() {

	static int prev_whls[2];
	static double x, y, tau;
	static std::chrono::time_point<std::chrono::steady_clock> prev_t;

	static bool initialized = false; 

	double r = .042 / 2.; // m
	double L = .055; // m   [Laumond1998] With designating the distance between the driving wheels the dynamic model is :
	const double pi = 3.14159265358979323846;

	auto &sensors = m_port_sensors.prepare();   // reference is critical 

	sensors.t = (unsigned long long) (m_robot->getTime() * 1000.0);

	sensors.b_ps = true; 
	for (int i = 0; i < sensors.N; i++) {
		sensors.ps[i] = m_robot->psValues[i];  
	}

	sensors.b_tof = true;
	sensors.tof = m_robot->tofValue;

	sensors.b_ls = true;
	for (int i = 0; i < sensors.N; i++) {
		sensors.ls[i] = m_robot->lsValues[i];
	}

	sensors.b_whls = true; 
	for (int i = 0; i < 2; i++) {
		sensors.whls[i] = m_robot->whlsValues[i];
	}

	// first 0 received indicats the stuff is working
	if (sensors.whls[0] == 0) {
		initialized = true;
	}

	// init delta calc
	if (!initialized) {
		prev_t = std::chrono::steady_clock::now();
		for (int i = 0; i < 2; i++) {
			prev_whls[i] = 0;
		}
		x = y = 0.165; 
		tau = pi / 4.; // 45° 
	}

	sensors.b_accl = true;
	char buf[50];
	for (int i = 0; i < 3; i++) {
		sensors.accl[i] = (int) (m_robot->accelValues[i] * 1000.0);   // mm/s^2
	}

	sensors.b_gyro = true;
	for (int i = 0; i < 3; i++) {
		sensors.gyro[i] = (int)m_robot->gyroValues[i];
	}

	m_port_sensors.write(); 

	if (initialized) {
		//using namespace std::literals;
		//const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
		const auto now = std::chrono::steady_clock::now();
		auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - m_robot->active);
		auto dms = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_t);

		fprintf(m_steps, "%lld;%d;%d\n", ms.count(), sensors.whls[0], sensors.whls[1]);

		// https://www.gctronic.com/doc/index.php?title=e-puck2_PC_side_development
		// 1000 steps per revolution,  steps/1000 * 360°  = z.B 42/1000 * 360 = 15°    rad -> 2 pi = 360

		//const double dt = 0.124; // ms / 1000.;  
		const double dt = double(dms.count()) / 1000.;
		const int dsteps_l = sensors.whls[0] - prev_whls[0];
		const int dsteps_r = sensors.whls[1] - prev_whls[1];
		const double v_l = dsteps_l / 1000. * 2. * pi * r / dt;  // Circumference (Kreisumfang) C = 2 pi r
		const double v_r = dsteps_r / 1000. * 2. * pi * r / dt;  // m/s
		const double v = (v_l + v_r) / 2.;
		const double w = (v_l - v_r) / L;
		const double dx = cos(tau) * v;
		const double dy = sin(tau) * v;
		const double dtau = w;

		// trajectory
		x += dx * dt;
		y += dy * dt;
		tau += dtau * dt;

		fprintf(m_trajectory, "%lld;%lf;%lf;%lf;%d;%d;%lf;%lf;%lf;%lf;%lf%;%lf;%lf\n",
			ms.count(),
			x, y, tau,
			dsteps_l, dsteps_r,
			v_l, v_r,
			v, w,
			dx, dy, dtau);

		// delta
		prev_t = now;
		for (int i = 0; i < 2; i++) {
			prev_whls[i] = sensors.whls[i];
		}

	}

}


bool CarlEpuck2::receiveActuatorOutput() {

	bool received = false; 

	EPuck2Actuators* actuators = m_port_actuators.read(true); // no 

	if (actuators) {
		received = true;

		for (int i = 0; i < actuators->N; i++) {
			m_robot->speeds[i] = actuators->speeds[i];
		}

	}

	return received;
}


void EPuck2ActuatorsReceiver::onRead(EPuck2Actuators& actuators)  {

	if (actuators.b_vel) {
		printf("[vel] %f %f ", actuators.speeds[0], actuators.speeds[1]);
		puck->m_robot->setSpeed(actuators.speeds[0], actuators.speeds[1]); 
	}

	if (actuators.b_red) {
		printf("[red] %d %d %d %d ", actuators.led[0], actuators.led[2], actuators.led[4], actuators.led[6]);
		puck->m_robot->setLedValues({0,2,4,6}, actuators.led);
	}

	if (actuators.b_rgb) {
		printf("[rgb] 0x%X 0x%X 0x%X 0x%X ", actuators.led[1], actuators.led[3], actuators.led[5], actuators.led[7]);
		puck->m_robot->setLedValues({1,3,5,7}, actuators.led);
	}

	if (actuators.b_frnt) {
		printf("[frnt] %d ", actuators.led[9]);
		puck->m_robot->setLedValues({9}, actuators.led);
	}

	if (actuators.b_chss) {
		printf("[chss] %d ", actuators.led[8]);
		puck->m_robot->setLedValues({8}, actuators.led);
	}

	if (actuators.b_spk) {
		printf("[spk] %s ", actuators.spk);
		puck->m_robot->setText(actuators.spk);
	}

	printf("\n");

}

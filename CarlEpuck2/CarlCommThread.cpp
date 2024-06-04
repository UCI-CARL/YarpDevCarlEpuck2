
#include <iostream>
#include <sstream>
#include <stdexcept>

#include <cstdlib>
#include <cstring>

//Linux Pi5
#include<thread>

using namespace std;

#ifdef _WIN32
#include <winsock.h>
#else 
#include <sys/socket.h>
#include <netinet/in.h>  // https://stackoverflow.com/questions/32596553/sockaddr-structure-sys-socket-h
#include <netdb.h>
#endif

#define SOCKET_PORT 1000

#include "CarlEpuck2.h"
#include "CarlEpuck2Robot.h"
#include "CarlCommThread.h"

using namespace yarp::dev;

using namespace yarp::os;
using namespace yarp::dev;


#define myTrace   if(m_log <= Log::TraceType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).trace
#define myDebug   if(m_log <= Log::DebugType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).debug
#define myInfo    if(m_log <= Log::InfoType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).info
#define myWarning if(m_log <= Log::WarningType) yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).warning
#define myError   if(m_log <= Log::ErrorType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).error
#define myFatal   if(m_log <= Log::FatalType)	yarp::os::Log(__FILE__, __LINE__, __YFUNCTION__).fatal

#define ifLogDebug   if(m_log <= yarp::os::Log::DebugType)	yarp::os::Log().debug()	

#include "EPuckCommandPacket.hpp"
#include "Communication.hpp"
#include "Wrapper.hpp"

//#include <webots/camera.h>
#include <webots/remote_control.h>
#include <webots/nodes.h>

CarlCommThread::CarlCommThread(yarp::carl::CarlEpuck2* robot) : m_epuck2(robot), mFd(0)
{
	m_log = m_epuck2->m_log;

#ifdef _WIN32  // initialize the socket api
	WSADATA info;
	int rc = WSAStartup(MAKEWORD(1, 1), &info);  // Winsock 1.1
	if (rc != 0) {
		fprintf(stderr, "Cannot initialize Winsock\n");
	}
#endif
}

CarlCommThread::~CarlCommThread()
{
	cleanup();

	m_epuck2 = nullptr; // dereference
}

void CarlCommThread::run() {
	//cout << __FUNCTION__ << "\n";

	initConnection("192.168.2.21");   // todo read Ip from param
	// check status

	connected();   // wifi led changes from green to off

	while (true) {

		//enableSensors(true);   // wifi led changes from off to blue 

		readyRead(); 

		// tof
		int dist = getDistance(); 
		m_epuck2->m_robot->tofValue = dist;

		uint16_t dist_cm = getDistanceCm();
		std::vector<int>ps(8); 
		readPsTo(ps); 
		for (int i = 0; i < 8; i++)
			m_epuck2->m_robot->psValues[i] = ps[i];

		printf("tof %d mm  ps %d %d %d %d %d %d %d %d\n", (int) dist, ps[0], ps[1], ps[2], ps[3], ps[4], ps[5], ps[6], ps[7] );

		// set actuators
		go(m_epuck2->m_robot->speeds[0], m_epuck2->m_robot->speeds[1]);

		std::this_thread::sleep_for(2 * 64ms); // 256

	}

}



void CarlCommThread::readPsTo(std::vector<int>& ps) {
	ps[0] = getIr0();
	ps[1] = getIr1();
	ps[2] = getIr2();
	ps[3] = getIr3();
	ps[4] = getIr4();
	ps[5] = getIr5();
	ps[6] = getIr6();
	ps[7] = getIr7();
}

int CarlCommThread::getIr(int i) {
	switch (i) {
		case 0: return getIr0();
		case 1: return getIr1();
		case 2: return getIr2();
		case 3: return getIr3();
		case 4: return getIr4();
		case 5: return getIr5();
		case 6: return getIr6();
		case 7: return getIr7();
	}
}




bool CarlCommThread::threadInit() {

	return true;
}

void CarlCommThread::threadRelease() {

	closeCommunication();


}



void CarlCommThread::init() {

}


void CarlCommThread::initConnection(const std::string& ip) {

	packet_index = 0;
	read_state = 0;

	initialize(ip); 

}

void CarlCommThread::closeCommunication() {
	img_count = 0;
	sensors_count = 0;

	cleanup();
}


void CarlCommThread::enableSensors(bool state) {
	if (state) {
		next_request |= 0x02;
	}
	else {
		next_request &= ~(0x02);
	}
	send_cmd = true;
}

void CarlCommThread::enableCamera(bool state) {
	if (state) {
		next_request |= 0x01;
	}
	else {
		next_request &= ~(0x01);
	}
	send_cmd = true;
}


// slot
void CarlCommThread::connected()
{

	output_buffer[0] = 0x80;
	output_buffer[1] = 0x02; // Bit0: start/stop image stream; bit1: start/stop sensors stream.
	output_buffer[2] = 0x00; // Behavior / others
	output_buffer[3] = 0x00; // Left speed LSB
	output_buffer[4] = 0x00; // Left speed MSB
	output_buffer[5] = 0x00; // Right speed LSB
	output_buffer[6] = 0x00; // Right speed MSB
	output_buffer[7] = 0x00; // LEDs
	output_buffer[8] = 0x00; // LED2 red
	output_buffer[9] = 0x00; // LED2 green
	output_buffer[10] = 0x00; // LED2 blue
	output_buffer[11] = 0x00; // LED4 red
	output_buffer[12] = 0x00; // LED4 green
	output_buffer[13] = 0x00; // LED4 blue
	output_buffer[14] = 0x00; // LED6 red
	output_buffer[15] = 0x00; // LED6 green
	output_buffer[16] = 0x00; // LED6 blue
	output_buffer[17] = 0x00; // LED8 red
	output_buffer[18] = 0x00; // LED8 green
	output_buffer[19] = 0x00; // LED8 blue
	output_buffer[20] = 0x00; // sound

	next_request = output_buffer[1];

	//socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
	send((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);

} 

// slot
void CarlCommThread::disconnected()
{

}

// slot
void CarlCommThread::bytesWritten(int64_t bytes)
{

}


// slot
void CarlCommThread::readyRead()
{

	if (packet_index == 0) {
		memset(input_buffer, 0x0, MAX_BUFF_SIZE);
	}

	while (socketBytesAvailable() > 0) {

		switch (read_state) {
		case 0: // Read header
			receive((char*)&input_buffer[0], 1); 

			if (input_buffer[0] == 0x01) {
				read_state = 1;
			}
			else if (input_buffer[0] == 0x02) {
				read_state = 2;
			}
			else if (input_buffer[0] == 0x03) {
				if (send_cmd) {
					send_cmd = false;
					output_buffer[1] = next_request;

					send((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
					output_buffer[20] = 0; // Clear sound value.
				}
			}
			packet_index = 0;
			break;

		case 1: // Read image
			packet_index += receive((char*)&input_buffer[packet_index], MAX_BUFF_SIZE - packet_index);

			if (packet_index == MAX_BUFF_SIZE) {
				packet_index = 0;
				img_count++;
				//emit newImage();
				if ((output_buffer[1] & 0x02) == 0x00) { // If only image is streamed.
					if (send_cmd) {
						send_cmd = false;
						output_buffer[1] = next_request;
						//qDebug() << "next req 1 = " << next_request;
						//socket->write((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
						send((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
						output_buffer[20] = 0; // Clear sound value.
					}
				}
				read_state = 0;
			}
			break;

		case 2: // Read sensors
			packet_index += receive((char*)&input_buffer[packet_index], INPUT_BUFFER_SIZE - packet_index);

			if (packet_index == INPUT_BUFFER_SIZE) {
				packet_index = 0;

				long  mantis = 0;
				short  exp = 0;
				float flt = 0;

				// Compute acceleration
				mantis = (input_buffer[6] & 0xff) + ((input_buffer[7] & 0xffl) << 8) + (((input_buffer[8] & 0x7fl) | 0x80) << 16);
				exp = (input_buffer[9] & 0x7f) * 2 + ((input_buffer[8] & 0x80) ? 1 : 0);
				if (input_buffer[9] & 0x80) {
					mantis = -mantis;
				}
				flt = (mantis || exp) ? ((float)ldexp (mantis, (exp - 127 - 23))) : 0;
				acceleration = flt;

				// Compute orientation.
				mantis = (input_buffer[10] & 0xff) + ((input_buffer[11] & 0xffl) << 8) + (((input_buffer[12] & 0x7fl) | 0x80) << 16);
				exp = (input_buffer[13] & 0x7f) * 2 + ((input_buffer[12] & 0x80) ? 1 : 0);
				if (input_buffer[13] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float)ldexp (mantis, (exp - 127 - 23))) : 0;
				orientation = flt;
				if (orientation < 0.0)
					orientation = 0.0;
				if (orientation > 360.0)
					orientation = 360.0;

				//qDebug() << "orientation = " << orientation;

				// Compute inclination.
				mantis = (input_buffer[14] & 0xff) + ((input_buffer[15] & 0xffl) << 8) + (((input_buffer[16] & 0x7fl) | 0x80) << 16);
				exp = (input_buffer[17] & 0x7f) * 2 + ((input_buffer[16] & 0x80) ? 1 : 0);
				if (input_buffer[17] & 0x80)
					mantis = -mantis;
				flt = (mantis || exp) ? ((float)ldexp (mantis, (exp - 127 - 23))) : 0;
				inclination = flt;
				if (inclination < 0.0)
					inclination = 0.0;
				if (inclination > 180.0)
					inclination = 180.0;

				//qDebug() << "inclination = " << inclination;

				// Gyro
				gyroRaw[0] = input_buffer[18] + input_buffer[19] * 256;
				gyroRaw[1] = input_buffer[20] + input_buffer[21] * 256;
				gyroRaw[2] = input_buffer[22] + input_buffer[23] * 256;

				// Magnetometer
				magneticField[0] = *((float*)&input_buffer[24]);
				magneticField[1] = *((float*)&input_buffer[28]);
				magneticField[2] = *((float*)&input_buffer[32]);

				// Temperature.
				//temperature = input_buffer[36];
				//qDebug() << "temp = " << temperature;

				// Proximity sensors data.
				ir0 = (input_buffer[37] + input_buffer[38] * 256 > 2000) ? 2000 : input_buffer[37] + input_buffer[38] * 256;
				ir1 = (input_buffer[39] + input_buffer[40] * 256 > 2000) ? 2000 : input_buffer[39] + input_buffer[40] * 256;
				ir2 = (input_buffer[41] + input_buffer[42] * 256 > 2000) ? 2000 : input_buffer[41] + input_buffer[42] * 256;
				ir3 = (input_buffer[43] + input_buffer[44] * 256 > 2000) ? 2000 : input_buffer[43] + input_buffer[44] * 256;
				ir4 = (input_buffer[45] + input_buffer[46] * 256 > 2000) ? 2000 : input_buffer[45] + input_buffer[46] * 256;
				ir5 = (input_buffer[47] + input_buffer[48] * 256 > 2000) ? 2000 : input_buffer[47] + input_buffer[48] * 256;
				ir6 = (input_buffer[49] + input_buffer[50] * 256 > 2000) ? 2000 : input_buffer[49] + input_buffer[50] * 256;
				ir7 = (input_buffer[51] + input_buffer[52] * 256 > 2000) ? 2000 : input_buffer[51] + input_buffer[52] * 256;
				if (ir0 < 0) {
					ir0 = 0;
				}
				if (ir1 < 0) {
					ir1 = 0;
				}
				if (ir2 < 0) {
					ir2 = 0;
				}
				if (ir3 < 0) {
					ir3 = 0;
				}
				if (ir4 < 0) {
					ir4 = 0;
				}
				if (ir5 < 0) {
					ir5 = 0;
				}
				if (ir6 < 0) {
					ir6 = 0;
				}
				if (ir7 < 0) {
					ir7 = 0;
				}

				// Compute abmient light.
				lightAvg += (input_buffer[53] + input_buffer[54] * 256);
				lightAvg += (input_buffer[55] + input_buffer[56] * 256);
				lightAvg += (input_buffer[57] + input_buffer[58] * 256);
				lightAvg += (input_buffer[59] + input_buffer[60] * 256);
				lightAvg += (input_buffer[61] + input_buffer[62] * 256);
				lightAvg += (input_buffer[63] + input_buffer[64] * 256);
				lightAvg += (input_buffer[65] + input_buffer[66] * 256);
				lightAvg += (input_buffer[67] + input_buffer[68] * 256);
				lightAvg = (int)(lightAvg / 8);
				lightAvg = (lightAvg > 4000) ? 4000 : lightAvg;
				if (lightAvg < 0) {
					lightAvg = 0;
				}

				// ToF
				distance = (uint16_t)(((uint8_t)input_buffer[70] << 8) | ((uint8_t)input_buffer[69]));
				distanceCm = distance / 10;
				memset(distanceCmStr, 0x0, 5);
				sprintf(distanceCmStr, "%d", (distanceCm > 200) ? 200 : distanceCm);

				// Microphone
				micVolume[0] = ((uint8_t)input_buffer[71] + (uint8_t)input_buffer[72] * 256 > 1500) ? 1500 : ((uint8_t)input_buffer[71] + (uint8_t)input_buffer[72] * 256);
				micVolume[1] = ((uint8_t)input_buffer[73] + (uint8_t)input_buffer[74] * 256 > 1500) ? 1500 : ((uint8_t)input_buffer[73] + (uint8_t)input_buffer[74] * 256);
				micVolume[2] = ((uint8_t)input_buffer[75] + (uint8_t)input_buffer[76] * 256 > 1500) ? 1500 : ((uint8_t)input_buffer[75] + (uint8_t)input_buffer[76] * 256);
				micVolume[3] = ((uint8_t)input_buffer[77] + (uint8_t)input_buffer[78] * 256 > 1500) ? 1500 : ((uint8_t)input_buffer[77] + (uint8_t)input_buffer[78] * 256);

				// left + right steps
				leftSteps = (uint8_t)input_buffer[79]+ (uint8_t)input_buffer[80]*256;
				rightSteps = (uint8_t)input_buffer[81]+ (uint8_t)input_buffer[82]*256;

				// Battery
				batteryRaw = (uint8_t)input_buffer[83] + (uint8_t)input_buffer[84] * 256;
				memset(batteryRawStr, 0x0, 5);
				sprintf(batteryRawStr, "%d", batteryRaw);

				// Micro sd state.
				microSdState = input_buffer[85];

				// Tv remote.
				irCheck = input_buffer[86];
				irAddress = input_buffer[87];
				irData = input_buffer[88];
				memset(irCheckStr, 0x0, 8);
				memset(irAddressStr, 0x0, 8);
				memset(irDataStr, 0x0, 8);
				sprintf(irCheckStr, "%x", irCheck);
				sprintf(irAddressStr, "%x", irAddress);
				sprintf(irDataStr, "%x", irData);

				// Selector.
				selector = input_buffer[89];
				memset(selectorStr, 0x0, 3);
				sprintf(selectorStr, "%d", selector);


				// Button state.
				buttonState = input_buffer[102];

				sensors_count++;
				//emit newBinaryData();
				if (send_cmd) {
					send_cmd = false;
					output_buffer[1] = next_request;
					send((char*)&output_buffer[0], OUTPUT_BUFFER_SIZE);
					output_buffer[20] = 0; // Clear sound value.
				}
				read_state = 0;
			}
			break;

		}

	}

}



void CarlCommThread::go(int speed_left, int speed_right) {
	output_buffer[3] = speed_left & 0xFF;
	output_buffer[4] = (speed_left >> 8) & 0xFF;
	output_buffer[5] = speed_right & 0xFF;
	output_buffer[6] = (speed_right >> 8) & 0xFF;
	send_cmd = true;
}



// -------------
// Wifi

bool CarlCommThread::initialize(const string& ip) {
	struct sockaddr_in address;
	struct hostent* server;
	int rc;
	mFd = socket(AF_INET, SOCK_STREAM, 0);
	if (mFd == -1) {
		fprintf(stderr, "Cannot create socket\n");
		return false;
	}
	memset(&address, 0, sizeof(struct sockaddr_in));
	address.sin_family = AF_INET;
	address.sin_port = htons(SOCKET_PORT);
	server = gethostbyname(ip.c_str());	
	
	if (server)
		memcpy(reinterpret_cast<char*>(&address.sin_addr.s_addr), reinterpret_cast<char*>(server->h_addr), server->h_length);
	else {
		fprintf(stderr, "Cannot resolve server name: %s\n", ip.c_str());
		cleanup();
		return false;
	}
	rc = connect(mFd, (struct sockaddr*)&address, sizeof(struct sockaddr));
	if (rc == -1) {
		fprintf(stderr, "Cannot connect to the server\n");
		cleanup();
		return false;
	}
	return true;
}


void CarlCommThread::cleanup() {
	if (mFd > 0)
#ifdef _WIN32
		closesocket(mFd);
#else
		close(mFd);
#endif
	mFd = 0;
}



bool CarlCommThread::send(const char* data, int size) {
	int n = 0;
	do {
		int m = ::send(mFd, &data[n], size - n, 0);
		if (m == -1)
			return false;
		n += m;
	} while (n < size);
	return n == size;
}

int CarlCommThread::receive(char* data, int size, bool block) {
	int n = 0;
	int flag;

#ifdef _WIN32

	if (!block) {
		u_long iMode = 0;
		if (ioctlsocket(mFd, FIONBIO, &iMode) != NO_ERROR)
			fprintf(stderr, "ioctlsocket failed\n");
	}
	flag = 0;

#else

	flag = block ? 0 : MSG_DONTWAIT;

#endif

	do {
		int m = ::recv(mFd, &data[n], size - n, flag);
#ifdef _WIN32
		if (m == SOCKET_ERROR) {
			if (WSAGetLastError() == WSAEWOULDBLOCK)
				return 0;
			else
				return -1;
		}
#else
		if (m == -1) {
			if (errno == EAGAIN || errno == EWOULDBLOCK)
				return 0;
			else
				return -1;
		}
#endif
		n += m;
	} while (n < size);
	return size;  // success
}




int64_t CarlCommThread::socketBytesAvailable() {
#ifdef _WIN32
	u_long iMode = 0;
	if (ioctlsocket(mFd, FIONREAD, &iMode) != NO_ERROR)
	{
		fprintf(stderr, "ioctlsocket failed\n");
		return -1;
	}
	else {
		return iMode;
	}
#else
	return -1;
#endif

}


void CarlCommThread::stopMotors() {
	output_buffer[3] = 0;
	output_buffer[4] = 0;
	output_buffer[5] = 0;
	output_buffer[6] = 0;
	send_cmd = true;
}


void CarlCommThread::setIp(const std::string& ip) {
	this->ip = ip; 
}

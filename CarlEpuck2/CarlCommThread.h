
#pragma once	// solves order related issues

#include <stdio.h>


#define IMAGE_BUFF_SIZE (4056+3)

#define MAX_BUFF_SIZE 38400 // Color QQVGA (160x120x2)
#define OUTPUT_BUFFER_SIZE 21
#define INPUT_BUFFER_SIZE 104

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
class CarlCommThread : public yarp::os::Thread
{
public:
	yarp::carl::CarlEpuck2* m_epuck2;
	yarp::os::Log::LogType m_log;
	
public:
	CarlCommThread(yarp::carl::CarlEpuck2* epuck2);
	virtual void run();
	virtual bool threadInit();
	virtual void threadRelease();

public:
	~CarlCommThread();

	void init();  // threadInit
	void initConnection(const std::string& ip = "");
	void closeCommunication();  // threadReleaes

	void getImg(unsigned char* img);
	void enableSensors(bool b);
	void enableCamera(bool b);

	int getDistance(void) {return distance; }
	uint16_t getDistanceCm(void) { return distanceCm; }
	char* getDistanceCmStr(void) { return distanceCmStr; }

	float getFps() { return fps; }
	float getSensorsRate() { return refresh_rate; }
	uint8_t buttonIsPressed(void) { return buttonState == 1; }
	uint8_t getMicroSdState(void) { return microSdState; }
	char* getSelector() { return selectorStr; }
	char* getIrCheck() { return irCheckStr; }
	char* getIrAddress() { return irAddressStr; }
	char* getIrData() { return irDataStr; }

	// Wifi.Communication
	bool initialize(const std::string& ip);
	bool isInitialized() const { return mFd > 0; }
	void cleanup();
	bool send(const char*, int size);
	int receive(char*, int size, bool block = true);
	void setIp(const std::string& ip);

	// Qt ports 
	// http://diranieh.com/SOCKETS/DosDonts.htm

	/* https://stackoverflow.com/questions/9108574/getting-number-of-bytes-available-to-read-in-a-socket
	On Windows at least, you can use ioctlsocket() with the FIONREAD command to determine the current number 
	of bytes that are available for recv() to read without blocking. By the time you actually call recv(), 
	more bytes may have arrived, though.*/
	int64_t socketBytesAvailable();  //  qint64 QIODevice::bytesAvailable() const

	float getAcceleration() { return acceleration; }
	float getOrientation() { return orientation; }
	float getInclination() { return inclination; }

	void readPsTo(vector<int>& ps);

	int getIr(int i);

	int getIr0() { return ir0; }
	int getIr1() { return ir1; }
	int getIr2() { return ir2; }
	int getIr3() { return ir3; }
	int getIr4() { return ir4; }
	int getIr5() { return ir5; }
	int getIr6() { return ir6; }
	int getIr7() { return ir7; }

	int getLight() { return lightAvg; }
	uint16_t getMic(uint8_t id) { return micVolume[id]; }
	void setSpeed(int16_t speed) { motorSpeed = speed; }
	int getType() { return type; }
	int getWidth() { return width; }
	int getHeight() { return height; }
	int getPixNum() { return pixNum; }
	void setImgType(int t) { type = t; }
	void setImgWidth(int w) { width = w; }
	void setImgHeight(int h) { height = h; }
	void setImgPixNum(int p) { pixNum = p; }
	uint16_t getBatteryRaw() { return batteryRaw; }
	char* getBatteryRawStr() { return batteryRawStr; }
	int16_t getGyroRaw(uint8_t axis) { return gyroRaw[axis]; }
	float getMagneticField(uint8_t axis) { return magneticField[axis]; }

	int getLeftSteps() { return leftSteps; }
	int getRightSteps() { return rightSteps; }

//private:
public:
	int mFd;
	std::string ip; // for reconnect
	int port; // reserved

	uint16_t packet_index;
	uint8_t input_buffer[MAX_BUFF_SIZE];
	uint8_t output_buffer[OUTPUT_BUFFER_SIZE];
	uint16_t img_count;
	//QImage img;
	uint8_t read_state;
	//QTimer* fpsTimer;
	float fps;
	uint16_t sensors_count;
	float refresh_rate;
	float acceleration, orientation, inclination;		/**< acceleration data*/
	int ir0, ir1, ir2, ir3, ir4, ir5, ir6, ir7;			/**< proximity sensors data*/
	int lightAvg;										/**< light sensor data*/
	uint16_t micVolume[4];								/**< microphone data*/
	uint16_t batteryRaw;
	char batteryRawStr[5];
	int16_t gyroRaw[3];
	uint16_t distance;
	uint16_t distanceCm;
	char distanceCmStr[5];
	uint8_t buttonState;
	uint8_t microSdState;
	uint8_t next_request;
	int16_t motorSpeed;
	bool send_cmd;
	float magneticField[3];
	int16_t leftSteps, rightSteps;
	uint8_t irCheck, irAddress, irData;
	uint8_t selector;
	uint8_t temperature;
	int16_t groundProx[3], groundAmbient[3];
	uint8_t rgbLedValue[3];
	int rgbLedState[4];
	char selectorStr[3];
	char irCheckStr[8], irAddressStr[8], irDataStr[8];
	int16_t acc_x, acc_y, acc_z;

	unsigned int type;						/**< type of the image: color (value 1) or grayscale (value 0)*/
	unsigned int width;						/**< width of the image to be received*/
	unsigned int height;					/**< height of the image to be received*/
	unsigned int pixNum;					/**< total number of pixels (bytes) to be received; in case of grayscale image it is width*height, in case of color image it is width*height*2 (RGB565)*/
	unsigned int zoom;

  public:  // slots:
	   void connected();
	   void disconnected();
	   void bytesWritten(int64_t bytes);
	   void readyRead();
	   void timerEvent();

	   void go(int speed_left, int speed_right); 

	   void goForward();
	   void goBackward();
	   void goLeft();
	   void goRight();
	   void stopMotors();
	   void led0Slot(int state);
	   void led1Slot(int state);
	   void led2Slot(int state);
	   void led3Slot(int state);
	   void led4Slot(int state);
	   void led5Slot(int state);
	   void led6Slot(int state);
	   void led7Slot(int state);
	   void led8Slot(int state);
	   void led9Slot(int state);
	   void sound1Slot();
	   void sound2Slot();
	   void sound3Slot();
	   void sound4Slot();
	   void sound5Slot();
	   void audioOffSlot();
	   void updateRed(int value);
	   void updateGreen(int value);
	   void updateBlue(int value);

	   void updateParameters(int t, int w, int h, int z);		/**< called when the "Send Parameters" button is clicked; send the command to the robot to change the camera parameters*/

	public: // signals:   // see emit
		void connectionClosed();
		void newImage();
		void updateUiState(uint8_t);
		void updateFps();
		void newBinaryData();
		void newAsciiData();
		void cannotOpenPort(string s);
		void showVersion(string s, int);
		void reconnect();

};
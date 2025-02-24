/*!
  \file    tcm command interface 
  \brief   modif for downloading arguments

  \version 2025-02-24, V1.0
	\author	 kevin.wang
	\note    none
*/
#include <cstdlib>
#include <cstdio>

#include <FastLED.h>

#include <elapsedMillis.h>
#include <CRC32.h>

const char gtitle[] = "TCM_args_download";
const char gversion[] = "V1.00";
char gtmpbuf[100];

// Teensy4.1 board v2 def

// RGB LED define
static const int LED_R = 36;
static const int LED_G = 37;
static const int LED_B = 38;

// laser driver en
static const int LASER_550nm = 19;
static const int LASER_402nm = 18;
static const int LASER_470nm = 17;
static const int LASER_638nm = 16;
static const int LASER_735nm = 15;

// despeckler
static const int Dispecker_pin   = 33;

// laser status read port
static const int STATUS_402nm_TTL = 31;
static const int STATUS_470nm_TTL = 30;
static const int STATUS_550nm_TTL = 32;
static const int STATUS_638nm_TTL = 29;
static const int STATUS_735nm_TTL = 28;

const int NUM_LASER_CHANNELS = 5;
const int NUM_TEMP_CHANNELS = 6;
const int NUM_VOLTAGE_CHANNELS = 6;
const int NUM_CURRENT_CHANNELS = 6;

const unsigned long EACH_QUERY_DISTANCE = 200UL; // 200 in milliseconds

enum CommandType{
	NONE = 0,
	COMMAND = 1
};

const uint8_t MAX_QUERY_TYPE_INDEX = 10;
CommandType query_frequnce[MAX_QUERY_TYPE_INDEX] = { COMMAND }; 
uint8_t current_query_type_index = 0;

const int8_t ERR_OUT_OF_RANGE = 100;

CRC32 crc;
// Pins for laser channels (adjust as needed)
const int laserPins[NUM_LASER_CHANNELS] = {LASER_402nm, LASER_470nm, LASER_638nm, LASER_735nm, LASER_550nm};
const int laserStatuspins[NUM_LASER_CHANNELS] = {STATUS_402nm_TTL, STATUS_470nm_TTL, STATUS_638nm_TTL, STATUS_735nm_TTL, STATUS_550nm_TTL};

// Temperature setpoints and channel states
float tempSetpoints[NUM_TEMP_CHANNELS] = {25.0, 25.0, 25.0, 25.0, 25.0, 88.6};
float tempCurrentPoints[NUM_TEMP_CHANNELS] = {0.6, 0.5, 0.4, 0.3, 0.2, 0.1};

// Volatge value of channels
float voltageCurrentPoints[NUM_VOLTAGE_CHANNELS] = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};

// current value of channels
float currentCurrentPoints[NUM_CURRENT_CHANNELS] = {0, 0, 0, 0, 0.1, 0.2};

elapsedMillis timeSinceLastActive;
elapsedMillis timeInErrorState[NUM_TEMP_CHANNELS] = {0};

elapsedMillis timeSinceLastQueryTemp = 0;
uint8_t gQueryTemperatureChannelIndex = 0;
uint8_t gQueryVoltageChannelIndex = 0;
uint8_t gQueryCurrentChannelIndex = 0;

// TCM modules protocol process variables
bool reply_frame_analyzing_flag = false;

char tcm_command_buf[256];
uint8_t tcm_command_buf_length = 0;

char tcm_reply_buf[256];
uint8_t tcm_reply_buf_length = 0;

// for judge whether the replay value is correct or not
char tcm_reply_title[256];
uint8_t tcm_reply_title_length = 0;
uint8_t tcm_reply_address = 0;
uint8_t tcm_reply_module = 0;
CommandType tcm_reply_command_type = NONE;

// host protocol process variables
uint8_t host_protocol_buf[256];
uint8_t host_protocol_buf_length = 0;

// record command from upstream host such as PC
char tcm_transparent_command_buf[255];
uint8_t tcm_transparent_command_length = 0; 

void setParameters(const uint8_t* buffer, size_t size) {
  if (size == 1 + NUM_TEMP_CHANNELS * sizeof(float)) {
    for (int i = 0; i < NUM_TEMP_CHANNELS; i++) {
      memcpy(&tempSetpoints[i], buffer + 1 + i * sizeof(float), sizeof(float));
    }
    sendACK();
  } else {
    sendNAK();
  }
}

void setCommand(const uint8_t* buffer, size_t size) {
	strncpy(tcm_transparent_command_buf, (const char *)(&buffer[1]), size);
	tcm_transparent_command_buf[size - 1] = '\0';
	tcm_transparent_command_length = size;

	getCMDACK();
}

/*
 transparent sent command from PC to TCM module	
 */
void tcmTransparentSend() {
	if ( tcm_transparent_command_length == 0 )
		return;

	// transparent send command to TMC
	sprintf(tcm_command_buf, "%s%c", tcm_transparent_command_buf, 0x0D);
	Serial5.write(tcm_command_buf, strlen(tcm_command_buf));

	// clear transparent command buffer length
	tcm_transparent_command_length = 0;

	tcm_reply_command_type = COMMAND;

	// reset tcm ptotocol analyzing buffer
	tcm_reply_buf_length = 0;

	// enable analyzing frame
	reply_frame_analyzing_flag = true;
}

/*
	upload data to upstream host, for example PC
 */
void uploadData(uint8_t* data, uint8_t length)
{
	Serial.write(data, length);
	// flag indicate one frame is over
	uint8_t end[2] = {0x0A, 0x0D};
	Serial.write(end, 2);
}

void sendACK() {
  uint8_t ack = 'A';
  uint32_t ackCRC = crc.calculate(&ack, 1);
  uint8_t ackPacket[5];
  ackPacket[0] = ack;
  memcpy(ackPacket + 1, &ackCRC, 4);
	uploadData(ackPacket, 5);
}

void sendNAK() {
  uint8_t nak = 'N';
  uint32_t nakCRC = crc.calculate(&nak, 1);
  uint8_t nakPacket[5];
  nakPacket[0] = nak;
  memcpy(nakPacket + 1, &nakCRC, 4);
	uploadData(nakPacket, 5);
}

void getCMDACK() {
  uint8_t ack = 'C';
  uint32_t ackCRC = crc.calculate(&ack, 1);
  uint8_t ackPacket[5];
  ackPacket[0] = ack;
  memcpy(ackPacket + 1, &ackCRC, 4);
	uploadData(ackPacket, 5);
}

/*
	send reply from TCM to upstream device such as PC
*/
void transparentCommandToUpstream(char* databuf, uint8_t datalength)
{
	if (datalength == 0)
		return;

  uint8_t statusPacket[1 + datalength + 1];
	statusPacket[0] = 'T';
	strncpy((char *)(&statusPacket[1]), databuf, datalength);
	statusPacket[1 + datalength] = '\0';

  uint32_t packetCRC = crc.calculate(statusPacket, sizeof(statusPacket));
  uint8_t finalPacket[sizeof(statusPacket) + 4];
  memcpy(finalPacket, statusPacket, sizeof(statusPacket));
  memcpy(finalPacket + sizeof(statusPacket), &packetCRC, 4);

	uploadData(finalPacket, sizeof(finalPacket));
}

void onPacketReceived(const uint8_t* buffer, size_t size) {
  if (size < 5) return; // Minimum packet size (1 byte command + 4 bytes CRC)

  uint32_t receivedCRC;
  memcpy(&receivedCRC, buffer + size - 4, 4);
  uint32_t calculatedCRC = crc.calculate(buffer, size - 4);

  if (receivedCRC != calculatedCRC) {
    sendNAK();
    return;
  }

  switch (buffer[0]) {
    case 'S': // Set parameters
      setParameters(buffer, size - 4);
      break;
    case 'C': // command need to be sent to TCM module 
      setCommand(buffer, size - 4);
      break;
  }
}

/*
	query loop, call it in the main loop
*/
void queryTCMDataMainLoop() {
	// each 200ms to query one channel data from TMC module
	if (timeSinceLastQueryTemp < EACH_QUERY_DISTANCE)
		return;
	timeSinceLastQueryTemp = 0;
	
	switch (query_frequnce[current_query_type_index++]) {
		case COMMAND:
			{
				tcmTransparentSend();
			}
			break;
		default:
			break;
	}

	if (current_query_type_index == MAX_QUERY_TYPE_INDEX)
		current_query_type_index = 0;
}

/*
	analyzing host(such as PC) frame
 */
void analyzingHostFrame() {
	if (Serial.available()) {
		host_protocol_buf[host_protocol_buf_length++] = Serial.read();
		if (host_protocol_buf[host_protocol_buf_length - 1] == 0x0D &&
				host_protocol_buf[host_protocol_buf_length - 2] == 0x0A) {
			onPacketReceived(host_protocol_buf, host_protocol_buf_length - 2);

			host_protocol_buf_length = 0;
		}
	}
}

/*
	analyzing TCM modules frame
 */
void analyzingTCMFrame() {
	if (reply_frame_analyzing_flag) {
		if (Serial5.available()) {
			tcm_reply_buf[tcm_reply_buf_length++] = Serial5.read();
			// read the end flag of frame
			if (tcm_reply_buf[tcm_reply_buf_length - 1] == 0x0D) {

				switch (tcm_reply_command_type) {
					case COMMAND:
						{
							transparentCommandToUpstream(tcm_reply_buf, tcm_reply_buf_length);
						}
						break;
					default:
						break;
				}

				// finish frame analyzing, reset flag
				reply_frame_analyzing_flag = false;
				tcm_reply_buf_length = 0;
				tcm_reply_address = 0;
				tcm_reply_module = 0;
			}
		}
	}
}

void setup() {
  Serial.begin(115200);
	delayMicroseconds(100000);	

	/*
	char cmd;
	while(1) {
		if (Serial.available()) {
			cmd = Serial.read();
			cmd = 'R';
			switch(cmd) {
				case 'R':
					goto Run;
					break;
			}
		}
	}
	*/

//Run:
	/*
  sprintf(gtmpbuf, "%s %s", gtitle, gversion);
  Serial.println(gtmpbuf);
	*/

  // TCM104x module UART5
  Serial5.begin(57600);
	delayMicroseconds(100000);	

  pinMode(LED_R, OUTPUT);
  digitalWrite(LED_R, LOW);

  pinMode(LED_G, OUTPUT);
  digitalWrite(LED_G, LOW);

  pinMode(LED_B, OUTPUT);
  digitalWrite(LED_B, LOW);

  digitalWrite(LED_G, HIGH);

  // enable pins
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserPins[i], OUTPUT);
    digitalWrite(laserPins[i], LOW);
  }

  pinMode(Dispecker_pin, OUTPUT);
  digitalWrite(Dispecker_pin, HIGH);
	
	// laser port start pins initialize
  for (int i = 0; i < NUM_LASER_CHANNELS; i++) {
    pinMode(laserStatuspins[i], INPUT);
  }

	timeSinceLastQueryTemp = 0;
}

void loop() {
	// query TCM module data
	queryTCMDataMainLoop();

	/*
  // code just for debug
	char cmd;
	if (Serial.available()) {
		cmd = Serial.read();

		switch (cmd) {
			case 'N':
				sendNAK();
				break;
			case 'A':
				sendACK();
				break;
			case 'T':
				Serial.println(tcm_transparent_command_buf);
				break;
			case 'R':
				for (int i = 0; i < 6; i++) {
					sprintf(gtmpbuf, "ch%d: %f", i, tempCurrentPoints[i]);
  				Serial.println(gtmpbuf);
				}
				break;
		}
	}
  // code just for debug
	*/

	// analyzing host frame
	analyzingHostFrame();

	// TCM modules protocol process
	analyzingTCMFrame();
}

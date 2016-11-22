/*
 * CAN interface to LIDAR Lite v3 via I2C
 * 
 * Client code expects to receive request for range via command :1
 * Client code sends back response in 2 bytes 
 * first byte is high 8, second byte is low 8 of 16 bit int
 * Client turns on LED when reading range, Turns led off once response is sent
 */

#include <LIDARLite.h>
#include <HID.h>
#include <SPI.h>
#include <Wire.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>

//*****************************************************************************
//
// The 32 bit values associated with the CAN_MSGID_API_STATUS request.
//
//*****************************************************************************
#define CAN_STATUS_CODE_M       0x0000ffff
#define CAN_STATUS_MFG_M        0x00ff0000
#define CAN_STATUS_DTYPE_M      0x1f000000
#define CAN_STATUS_CODE_S       0
#define CAN_STATUS_MFG_S        16
#define CAN_STATUS_DTYPE_S      24
#define CAN_MSGID_API_S         6 // Shifter for messageID

//*****************************************************************************
//
// The Reserved manufacturer identifiers in the Message Id.
//
//*****************************************************************************
#define CAN_MSGID_MFR_NI        0x00010000
#define CAN_MSGID_MFR_LM        0x00020000
#define CAN_MSGID_MFR_DEKA      0x00030000

//*****************************************************************************
//
// The Reserved device type identifiers in the Message Id.
//
//*****************************************************************************
#define CAN_MSGID_DTYPE_BCAST   0x00000000
#define CAN_MSGID_DTYPE_ROBOT   0x01000000
#define CAN_MSGID_DTYPE_MOTOR   0x02000000
#define CAN_MSGID_DTYPE_RELAY   0x03000000
#define CAN_MSGID_DTYPE_GYRO    0x04000000
#define CAN_MSGID_DTYPE_ACCEL   0x05000000
#define CAN_MSGID_DTYPE_USONIC  0x06000000
#define CAN_MSGID_DTYPE_GEART   0x07000000
#define CAN_MSGID_DTYPE_UPDATE  0x1f000000

#define CAN_STD                 0x0
#define CAN_EXT                 0x1

// Judgement Call Designed CAN components
#define CAN_MSGID_MFR_JUDGE     CAN_MSGID_MFR_LM

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int ZERO = 0;
const int SPI_CS_PIN = 9;
const int LED = 8;
/* Check the datasheets for your device for the arbitration IDs of the
messages you want to send.  By convention, this is a bitstring
containing the model number, manufacturer number, and api number. */
const INT32U REQUEST_RANGE_ARB_ID = 0x1 << CAN_MSGID_API_S;
const INT32U RESPOND_RANGE_ARB_ID = 0x2 << CAN_MSGID_API_S;
/*  Device ID, from 0 to 63. */
const INT32U MY_CAN_ID = 8 | CAN_MSGID_MFR_JUDGE | CAN_MSGID_DTYPE_MOTOR;

MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
LIDARLite myLidarLite;
boolean LEDState = false;
int LEDTimeOn = 0;

void initializeCAN() {
    while (CAN_OK != CAN.begin(CAN_1000KBPS))   // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println(" Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
}

void setup() {
	Serial.begin(115200);
	pinMode(LED, OUTPUT);
    initializeCAN();

	/*
	 begin(int configuration, bool fasti2c, char lidarliteAddress)

	 Starts the sensor and I2C.

	 Parameters
	 ----------------------------------------------------------------------------
	 configuration: Default 0. Selects one of several preset configurations.
	 fasti2c: Default 100 kHz. I2C base frequency.
	 If true I2C frequency is set to 400kHz.
	 lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	 operating manual for instructions.
	 */

	Serial.println("Configuring LIDAR");
	myLidarLite.begin(0, true); // Set configuration to default and I2C to 400 kHz

	/*
	 configure(int configuration, char lidarliteAddress)

	 Selects one of several preset configurations.

	 Parameters
	 ----------------------------------------------------------------------------
	 configuration:  Default 0.
	 0: Default mode, balanced performance.
	 1: Short range, high speed. Uses 0x1d maximum acquisition count.
	 2: Default range, higher speed short range. Turns on quick termination
	 detection for faster measurements at short range (with decreased
	 accuracy)
	 3: Maximum range. Uses 0xff maximum acquisition count.
	 4: High sensitivity detection. Overrides default valid measurement detection
	 algorithm, and uses a threshold value for high sensitivity and noise.
	 5: Low sensitivity detection. Overrides default valid measurement detection
	 algorithm, and uses a threshold value for low sensitivity and noise.
	 lidarliteAddress: Default 0x62. Fill in new address here if changed. See
	 operating manual for instructions.
	 */
	myLidarLite.configure(0); // Change this number to try out alternate configurations
	Serial.println("Done configuring LIDAR");

}

void turnLEDOff() {
	LEDState = false;
	digitalWrite(LED, LOW);
}

void turnLEDOn() {
	LEDState = true;
	LEDTimeOn = millis();
	digitalWrite(LED, HIGH);
}

const int respLen = 2;
INT8U stmp[respLen] = { 0, 0 };

void loop() {
	unsigned char len = 0;
	INT8U buf[8];

	if (CAN_MSGAVAIL == CAN.checkReceive()) {           // check if data coming
 		CAN.readMsgBuf(&len, buf); // read data,  len: data length, buf: data buf
		INT32U canId = CAN.getCanId();

        if (canId != 0x8041C7F) {
            Serial.println("-----------------------------");
            Serial.println("get data from ID: ");
            Serial.println(canId, HEX);
            Serial.println("want data from ID: ");
            Serial.println(MY_CAN_ID|REQUEST_RANGE_ARB_ID, HEX);
            Serial.print("Extended: ");
            Serial.println(CAN.isExtendedFrame(), HEX);
        }

		if (1) {
		//if (canId == (MY_CAN_ID|REQUEST_RANGE_ARB_ID)) {

            
			for (int i = 0; i < len; i++) {   // print the data
				Serial.print(buf[i]);
				Serial.print("\t");
				if (i == 0 && buf[i] == 1) {
					turnLEDOn();
					// Take a measurement with receiver bias correction and print to serial terminal
					int dist = myLidarLite.distance();
					Serial.print("Distance measured: ");
					Serial.println(dist);

					Serial.println("Sending data via CAN");
					stmp[0] = (dist & (ZERO | 0xFF00)) >> 2; // Shift upper bits to lower 8 bits and mask out high bits.
					stmp[1] = (dist & (ZERO | 0xFF)); // Mask out high bits to get only lower 8 bits
					Serial.print("Sending: ");
					Serial.print(stmp[0], HEX);
					Serial.println(stmp[1], HEX);
					if (CAN.sendMsgBuf(MY_CAN_ID|RESPOND_RANGE_ARB_ID, CAN_EXT, respLen, stmp) == CAN_OK) {
						Serial.println("Transmission SUCCESS!");
						for (int j = 0; j < 2; j++) {
							delay(250);
							turnLEDOff();
							delay(250);
							turnLEDOn();
						}
					} else {
						Serial.println("Transmission FAILED!");
                        initializeCAN();
						for (int j = 0; j < 4; j++) {
							delay(250);
							turnLEDOff();
							delay(250);
							turnLEDOn();
						}
					}
					turnLEDOff();
				}
			}
			Serial.println();
            //attachInterrupt(0, MCP2515_ISR, FALLING); // start interrupt
		}
	} else if (LEDTimeOn + 1000 < millis()) {
		// Turn LED off if it has been on for more than 1 sec
		turnLEDOff();
	}
}

/*********************************************************************************************************
 END FILE
 *********************************************************************************************************/

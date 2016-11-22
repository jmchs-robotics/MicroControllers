#include <rgb_lcd.h>
#include <mcp_can_dfs.h>
#include <mcp_can.h>
#include <SPI.h>


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


// Judgement Call Designed CAN components
#define CAN_MSGID_MFR_JUDGE     CAN_MSGID_MFR_LM
#define CAN_STD                 0x0
#define CAN_EXT                 0x1
// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
const int SPI_CS_PIN = 9;
const INT8U GET_RANGE = 0x1;/* Check the datasheets for your device for the arbitration IDs of the
messages you want to send.  By convention, this is a bitstring
containing the model number, manufacturer number, and api number. */
const INT32U REQUEST_RANGE_ARB_ID = 0x1 << CAN_MSGID_API_S;
const INT32U RESPOND_RANGE_ARB_ID = 0x2 << CAN_MSGID_API_S;
/*  Device ID, from 0 to 63. */
const INT32U LIDAR_ID = 8 | CAN_MSGID_MFR_JUDGE | CAN_MSGID_DTYPE_MOTOR;

MCP_CAN CAN(SPI_CS_PIN); // Set CS pin

INT8U buf[MAX_CHAR_IN_MESSAGE];
unsigned long lastRequest = 0;

// LCD vars
rgb_lcd lcd;
const int colorR = 255;
const int colorG = 0;
const int colorB = 0;

void setupLCD() {
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    lcd.setRGB(colorR, colorG, colorB);
    lcd.print("CAN_LiDAR Online");
}

void setup() {
	Serial.begin(115200);

	while (CAN_OK != CAN.begin(CAN_1000KBPS))   // init can bus : baudrate = 500k
	{
		Serial.println("CAN BUS Shield init fail");
		Serial.println(" Init CAN BUS Shield again");
        lcd.print("CAN Init Retry");
		delay(100);
	}
	Serial.println("CAN BUS Shield init ok!");
    setupLCD();
}

void getRange() {
	// send data:  id = 0x00, standard frame, data len, buf: data buf
	Serial.println("Requesting Range");
	buf[0] = GET_RANGE;
    Serial.print("Message Id: ");
    Serial.println(LIDAR_ID|REQUEST_RANGE_ARB_ID, HEX);
	if (CAN.sendMsgBuf(LIDAR_ID|REQUEST_RANGE_ARB_ID, CAN_EXT, 1, buf) == CAN_OK) {
		Serial.println("Transmission SUCCESS!");
	} else {
		Serial.println("Transmission FAILED!");
	}
}

void loop() {
	INT8U len = 0;
	INT32U canId = 0;

	// send data:  id = 0x00, standard frame, data len, buf: data buf
	if (lastRequest + 5000 < millis()) {
		lastRequest = millis();
		Serial.print("Next Request: ");
		Serial.println(lastRequest + 5000);
		getRange();
	} else {
		// check if data coming
		if (CAN_MSGAVAIL == CAN.checkReceive()) {
			CAN.readMsgBufID(&canId, &len, buf); // read data,  len: data length, buf: data buf

			Serial.println("-----------------------------");
			Serial.println("get data from ID: ");
			Serial.println(canId);

			if (canId == (LIDAR_ID|RESPOND_RANGE_ARB_ID)) {
				for (int i = 0; i < len; i++) {   // print the data
					Serial.print(buf[i], HEX);
					Serial.print("\t");
				}
				Serial.println();

				int range = (buf[0] << 2) + buf[1];

				Serial.print("Range read: ");
				lcd.clear();
				lcd.setCursor(0,0);
				lcd.print("Range read: ");
				lcd.setCursor(strlen("Range read: "), 0);
				lcd.print(range);
				Serial.println(range);
			}
		}
	}
}

/*********************************************************************************************************
 END FILE
 *********************************************************************************************************/

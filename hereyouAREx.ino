/************************************************************************************
 *
 * hereyouARE - Xadow version
 * AgileRescueEngine
 *
 * It needs the Xadow libraries to compile:
 * https://github.com/Seeed-Studio/Xadow_MainBoard
 * 
 * Functions:
 *
 * send SMS with the following format to trigger functions
 *
 * TRACK#ON#<sec>     Sends SMS with GPS position every <sec> to Primary Number
 * TRACK#OFF          Stops track functions
 * UBIDOTS#ON         Sets the Ubidots tracking ON or OFF
 * GETPOS#            Get the position via SMS instead of via call (useful when out of GSM coverage)
 * GETPOSGSM#         Get the position vis GSM cells (useful when out of GPS coverage)
 * INFO#              Sends debug infos to calling number
 *
 * Programming:
 *
 * send SMS with the following format to program the device
 *
 * PN#<num>         Sets the accepted number for receiving call and programming
 * Once set, other numbers will be ignored
 * A1#<num>         Sets additional numbers for programming/call acceptance
 * A2#<num>         Sets additional numbers for programming/call acceptance
 * A3#<num>         Sets additional numbers for programming/call acceptance
 * A4#<num>         Sets additional numbers for programming/call acceptance
 * SE#<string>      Sets the search engine for coordinates sents back with SMS
 * MU#METRIC        Sets the measure unit: METRIC or IMPERIAL
 * SA#x1;y1#x2;y2   Defines coordinates for Safe Area. Decimal format. Example:
 * 45.8834;9.8172#45.7456;10.2401
 * SF#OFF              Sets the Safe Area check: ON or OFF
 * SM#OFF              Sets the sleep Mode: ON or OFF
 * UT#<string>         Ubidots Auth Token (short token)
 * UI#<string>          Ubidots Variable ID
 * UA#<string>         APN for Ubidots connection
 *
 *
 * RESET
 * =====
 *
 * Press WAKE button for 15 seconds to reset the Primary Number
 *
 * LONET INIT
 * =========
 * AT+IPR=9600
 * AT+ECHARGE=1
 * AT&W
 *
 *
 * AVRDUDE
 * =======
 *
 * ./avrdude -C ../etc/avrdude.conf -c avrisp -b 19200 -P /dev/ttyACM0 -p m32u4 -v
 *
 * changed lfuse (original 0xFF, new 0xF9) to 1Mhz
 *
 * ./avrdude -C ../etc/avrdude.conf -c avrisp -b 19200 -P /dev/ttyACM0 -p m32u4 -U lfuse:w:0xf9:m
 *
 * to program directly with avrdude:
 *
 * avrdude -c avr109 -p atmega32u4 -P/dev/ttyACM0 -b57600 -Uflash:w:<filename.hex>:i
 *
 * $Id: hereyouAREx.ino,v 2.0 2016/05/15 13:57:51 cesare Exp cesare $
 *
 * TODO:
 * - Test safe are with 4 digits
 *
 ***************************************************************************************/

#include <Wire.h>
#include <stdlib.h>
#include <avr/wdt.h>
#include <EEPROM.h>
#include "xadow.h"

// Define the debug level (can not be enabled now, because running out of flash memory)
//#define DEBUG
#define UBIDOTS

#define MAXEEPROM        511
#define LASTWARNADDR     500
#define MAGICADDR        490
#define MAXBUFFER         40
#define MAXATBUFFER     130
#define MAXGPSBUFFER    130
#define MAXCONFLINES   14
#define MAXTELDIGIT       25
#define MAXALLOWED      4
#define WAKEPIN               A3     // It was 10 on Xadow
#define SLEEPGPSON      600000
#define SLEEPPAUSE      7000

// Service variables
boolean batteryWarnSent = false;
boolean resendSMS = false;
boolean safeAreaOn = false;
boolean trackOn = false;
boolean sleepMode = false;
boolean previousSleepMode = false;
boolean isGPSOn = false;
unsigned long gpsOnTime = 0;
long trackSec = 0;
unsigned long trackLast = 0;
unsigned long factoryResetLast = 0;
boolean factoryResetBegin = false;
const long magic = 19171107;
char trackNum[MAXTELDIGIT];
char factoryReset = 1;
int debug = 4;

#ifdef UBIDOTS
boolean ubiOn = false;
#endif

// Variables for GPS
char buff[MAXBUFFER];
char replybuffer[MAXATBUFFER];
char gpsbuff[MAXGPSBUFFER];
double currentLat, currentLong;

int atTimeout = 300;

/*************************************************************************/
/* Configuration loaded variables section                                */
/*************************************************************************/
// Variables for SMS
char searchEng_conf[MAXBUFFER] = "http://maps.google.com/?q=";

// Allowed numbers
char primaryNumber_conf[MAXTELDIGIT] = "+1234567890";
char allowedNumbers_conf[MAXALLOWED][MAXTELDIGIT] = {
	"+12345678901", "+12345678902", "+12345678903", "+12345678904"
};

char safeOnOf_conf[MAXBUFFER] = "OFF";
char measureUnit_conf[MAXBUFFER] = "METRIC";
char sleepMode_conf[MAXBUFFER] = "OFF";

// Safe area
int safeAreaWarnSent = 0;
char safeArea_conf[MAXBUFFER] = "0;0#0;0";
double safeAreaLat1, safeAreaLong1, safeAreaLat2, safeAreaLong2;

#ifdef UBIDOTS
// Ubidots variable
char ubiToken_conf[MAXBUFFER] = "NA";
char ubiIdVariable_conf[MAXBUFFER] = "NA";
char ubiAPN_conf[MAXBUFFER] = "NA";

unsigned long ubidotsLast = 0;
// Define the interval between every ubidots update (in milliseconds)
#define UBICHECK      60000
#endif

/*************************************************************************/

void setup()
{
	// Disable watchdog
	wdt_disable();

	// initialize serial communications
	Serial.begin(9600);
	Serial1.begin(9600);

	// Uncomment for debugging startup
	// while ( !Serial.available() ) ;

	Serial.println(F("--> Starting UP hereyouARE..."));

	// Initialize Xadow
	Xadow.init();

	// Load device configuration
	loadConf();

	// Enable watchdog
	wdt_enable(WDTO_8S);

	// Pat the watchdog
	wdt_reset();

	// Turn on GSM module
	turnGSMOn();

	// Turn on/off GPS module
	if ( sleepMode == false )
	{
		turnGPSOn();
	}
	else
	{
		turnGPSOff();
	}
	previousSleepMode = sleepMode;

	// Initialize WAKE button (will be used for factory reset)
	pinMode(WAKEPIN, INPUT);

#ifdef DEBUG
	if ( debug >= 1 ) Serial.println(F("--> GSM Powered on, and waiting ..."));
#endif
	delay(3000);
}

void loop()
{
	int batteryLevel;
	int charge;

	char GPSCoord[MAXBUFFER];
	char GPSAltitude[MAXBUFFER];
	char GPSSpeed[MAXBUFFER];
	int GPSSatNum;
	int GPSFix;
	unsigned long GPSDate;
	char *saveptr;

	// Initialize variables
	GPSFix = 0;
	charge = -1;
	batteryLevel = -1;
	GPSDate = 0;
	GPSSatNum = 0;

	// Variables for GSM
	char numTel[MAXBUFFER]; // buffer for the incoming call

	// Pat the watchdog
	wdt_reset();

	// Check the status of SleepMode
	if ( sleepMode != previousSleepMode )
	{
		if ( sleepMode == false )
		{
			turnGPSOn();
		}
		else
		{
			turnGPSOff();
		}
	}
	previousSleepMode = sleepMode;

	if ( isGPSOn == true )
	{
		// Read GPS position
		getGPS();
		GPSFix = parseCGNSINF(gpsbuff, GPSCoord, GPSAltitude, &GPSSatNum,&GPSDate,GPSSpeed);
	}

#ifdef UBIDOTS
	if ( ubiOn == true )
	{
		if ( (( millis() - ubidotsLast ) > UBICHECK) && GPSFix != 0 )
		{
			ubidotsSaveValue(GPSCoord);
			ubidotsLast = millis();
		}
	}
#endif

	////////////////////////////////////////
	// Check the status of the incoming call
	////////////////////////////////////////
	wdt_reset();
	getATReply(F("AT+CPAS"), 2500);
	wdt_reset();

	if  ( strncasecmp(replybuffer,"+CPAS: 3", MAXATBUFFER - 1) == 0 ||
	      strncasecmp(replybuffer,"RING", MAXATBUFFER - 1) == 0 )
	{

#ifdef DEBUG
		if ( debug >= 1 ) Serial.println(F("--> RECEIVING CALL"));
#endif

		// Retrieve the calling number
		getATReply(F("AT+CLCC"), atTimeout);
		strtok_r(replybuffer, "\"", &saveptr);
		strncpy(numTel, strtok_r(NULL, "\"", &saveptr), MAXBUFFER - 1);

#ifdef DEBUG
		// Print the calling number
		if ( debug >= 1 )
		{
			Serial.print(F("--> Number:"));
			Serial.println(numTel);
		}
#endif

		// Hang up
		getATReply(F("ATH"), atTimeout);

		// Send message with position
		if ( isAllowedNumber(numTel) )
		{
			if ( sleepMode == false )
			{
				sendPosSMS(numTel, GPSCoord, GPSAltitude, GPSSpeed, GPSSatNum, GPSFix);
				if ( GPSFix == 0 )
				{
					// If the SMS is sent with poor GPS signal, schedule a
					// re-send when signal will be stronger
					resendSMS = true;
				}
			}
			else
			{
				// Sleep mode is on, delay the SMS
				turnGPSOn();
				resendSMS = true;
			}
		}
	}

	Xadow.greenLed(LEDON);
	delay(20);
	// FIXME
	if ( isGPSOn == false ) Xadow.greenLed(LEDOFF);
	delay(500);

	// Resend SMS if the previous one had no fix
	if ( resendSMS == true && GPSFix != 0 )
	{
		sendPosSMS(numTel, GPSCoord, GPSAltitude, GPSSpeed, GPSSatNum, GPSFix);
		resendSMS = false;
	}

	// Check SMS presence
	checkInboxSMS();

	// Check Safe Area
	if ( safeAreaOn == true && GPSFix != 0 ) // FIXME
	{
		checkSafeArea();
	}

	// Check if Track function is on
	if ( trackOn == true )
	{
		if ( ( millis() - trackLast ) > trackSec * 1000 )
		{

			sendPosSMS(trackNum, GPSCoord, GPSAltitude, GPSSpeed, GPSSatNum, GPSFix);

			trackLast = millis();
#ifdef DEBUG
			if ( debug >= 1 )
			{
				Serial.println(F("--> Sending tracking message"));
			}
#endif
		}
	}

	// Get battery status
	getBatteryState(&charge, &batteryLevel);

	if ( charge == 1 && batteryLevel != 100 )
	{
		Xadow.redLed(LEDON);
	}
	else
	{
		Xadow.redLed(LEDOFF);
	}

	// Check if factory reset (WAKE) has been pressed
	factoryReset = digitalRead(WAKEPIN);
	if ( factoryReset == LOW && factoryResetBegin == false )
	{
		factoryResetLast = millis();
		factoryResetBegin = true;
	}
	else
	{
		if ( factoryReset == HIGH )
		{
			factoryResetLast = 0;
			factoryResetBegin = false;
		}
	}
	// If pressed for more than 15 seconds, reset configuration
	if ( factoryResetLast != 0 && millis() - factoryResetLast > 15000 )
	{
		// Reset primary number configuration
		strncpy(primaryNumber_conf, "+1234567890", MAXTELDIGIT - 1);
		writeConf();
	}

	// Check battery level
	if ( batteryLevel > 0 && batteryLevel < 35 && batteryWarnSent == false && charge != 1 )
	{
		sendSMS(primaryNumber_conf, "WARNING: Low battery!", false);
		batteryWarnSent = true;
	}

	// Manage sleepMode, turns off GPS after a while if Ubidots, track or safe area are off
	if ( sleepMode == true && ( ubiOn == false || trackOn == false ) )
	{
		if ( isGPSOn == true )
		{
			// Check if enough time has been passed
			if ( ( ( millis() - gpsOnTime ) > SLEEPGPSON ) && resendSMS != true )
			{
				turnGPSOff();
				GPSFix = 0;
				GPSSatNum = 0;
				Serial.println("--> GPS Turned Off"); // FIXME

			}
		}
		else
		{
			// Go in sleep mode...but only if not charging
			wdt_reset();

			if ( charge != 1 )
			{
				Serial1.end();

				Xadow.pwrDown(SLEEPPAUSE);
				Xadow.wakeUp();

				// Enable watchdog
				wdt_enable(WDTO_8S);
				Serial1.begin(9600);
			}
			else
			{
				delay(SLEEPPAUSE);
			}
		}
	}

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.print(F("--> Battery level: "));
		Serial.println(batteryLevel);
		Serial.print(F("--> Free RAM: "));
		Serial.println(freeRam());
	}
#endif
}

/*************************************************************************/
/* DEBUG Functions section                                               */
/*************************************************************************/

#ifdef DEBUG
int freeRam(void)
{
	extern int __heap_start, *__brkval;
	int v = 0;
	return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}
#endif

/*************************************************************************/
/* GSM Functions section                                                 */
/*************************************************************************/
void sendPosSMS(const char* tel, const char* coord, const char* alt, const char* gpsSpeed, int satNum, int gpsFix)
{
	int tmpSpeed;
	int tmpAlt;
	char tmpBuff[20];

	// Compose and send SMS
	getATReply(F("AT+CMGF=1"), atTimeout); // Select TEXT mode
	Serial1.print(F("AT+CMGS=\""));
	sprintf(tmpBuff, "%s", tel);
	Serial1.print(tmpBuff);
	Serial1.println("\"");
	delay(100);
	Serial1.print(F("hereyouARE - I'm here:\n"));
	Serial1.print("\"");
	Serial1.print(searchEng_conf);
	Serial1.print(coord);
	Serial1.print("\"");
	Serial1.print("\n");
	Serial1.print(F("Altitude: "));
	// Manage Altitude conversion
	tmpAlt = atoi(alt);
	if ( strcasecmp(measureUnit_conf, "METRIC") != 0 )
	{
		// Convert altitude in foot
		sprintf(tmpBuff, "%d", int(tmpAlt * 3.2808));
		Serial1.print(tmpBuff);
		Serial1.print(F(" feet"));
	}
	else
	{
		// Print altitude in meters
		sprintf(tmpBuff, "%d", tmpAlt);
		Serial1.print(tmpBuff);
		Serial1.print(F(" meters"));
	}
	Serial1.print("\n");

	Serial1.print(F("Speed: "));
	// Manage speed conversion
	tmpSpeed = atoi(gpsSpeed);
	if ( strcasecmp(measureUnit_conf, "METRIC") != 0 )
	{
		// Convert speed in mph
		sprintf(tmpBuff, "%d", int(tmpSpeed * 0.6213));
		Serial1.print(tmpBuff);
		Serial1.print(F(" Mph"));
	}
	else
	{
		// Print speed in Kmh
		sprintf(tmpBuff, "%d", tmpSpeed);
		Serial1.print(tmpBuff);
		Serial1.print(F(" Kmh"));
	}
	Serial1.print("\n");

	if ( gpsFix > 0 )
	{
		Serial1.print(F("Number of Sat: "));
		sprintf(tmpBuff, "%d", satNum);
		Serial1.print(tmpBuff);
	}
	else
	{
		Serial1.print(F("No sat. Position will be re-sent when available"));
	}

	delay(100);
	getATReply(F("\x1A"), atTimeout);

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> SMS is sent"));
	}
#endif
}

void sendGsmPosSMS(const char* tel, const char* mcc, const char* mnc, const char* cellid, const char* lac)
{
	char tmpBuff[20];

	// Compose and send SMS
	getATReply(F("AT+CMGF=1"), atTimeout); // Select TEXT mode
	Serial1.print(F("AT+CMGS=\""));
	sprintf(tmpBuff, "%s", tel);
	Serial1.print(tmpBuff);
	Serial1.println("\"");
	delay(100);
	Serial1.print(F("hereyouARE - GSM Cell ID:\n"));
	Serial1.print(F("MCC: "));
	Serial1.println(mcc);
	Serial1.print(F("MNC: "));
	Serial1.println(mnc);
	Serial1.print(F("CELLID: "));
	Serial1.println(cellid);
	Serial1.print(F("LAC: "));
	Serial1.println(lac);
	Serial1.println();
	Serial1.print(F("http://opencellid.org/#action=locations.cell&mcc="));
	Serial1.print(mcc);
	Serial1.print(F("&mnc="));
	Serial1.print(mnc);
	Serial1.print(F("&lac="));
	Serial1.print(lac);
	Serial1.print(F("&cellid="));
	Serial1.println(cellid);

	delay(100);
	getATReply(F("\x1A"), atTimeout);

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> SMS is sent"));
	}
#endif
}


void sendSMS(const char* tel, const char* msg, boolean INFO)
{
	// Compose and send SMS
	getATReply(F("AT+CMGF=1"), atTimeout); // Select TEXT mode
	Serial1.print(F("AT+CMGS=\""));
	Serial1.print(tel);
	Serial1.println(F("\""));

	Serial1.println(msg);

	// If this is a INFO SMS, attach the configuration and other info
	if ( INFO == true)
	{
		int addr = 0;

		// Read first char from eeprom
		char b = EEPROM.read(addr); // Read the first byte

		// Print the EEPROM content
		while ( b != 0xFF && addr < MAXEEPROM )
		{
			Serial1.print(b);
			addr++;
			b = EEPROM.read(addr);
		}
	}

	delay(100);
	Serial1.println();
	Serial1.write(0x1A);

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> SMS is sent"));
	}
#endif
}

int checkInboxSMS()
{
	int i = 0;
	char msg[MAXBUFFER];
	char *idxfld, *idx, *numberfld, *number;
	char num[MAXTELDIGIT];
	char *saveptr;
	char *saveptr2;

	getATReply(F("AT+CMGF=1"), atTimeout); // Select TEXT mode
	// Read all the messages
	Serial1.println(F("AT+CMGL=\"ALL\""));
	readPhoneLine(atTimeout, false);

	// If we have messages, we start to process them
	while ( strstr(replybuffer,"+CMGL") != NULL )
	{
		// Pat the watchdog
		wdt_reset();

		idxfld = strtok_r(replybuffer, ",", &saveptr);
		numberfld = strtok_r(NULL, ",", &saveptr); // Discard the 2nd value
		numberfld = strtok_r(NULL, ",", &saveptr); // This is the number which sent the message
		number = strtok_r(numberfld, "\"", &saveptr2);
		strncpy(num, number, MAXTELDIGIT - 1);

		// Get index of the message
		strtok_r(idxfld, " ", &saveptr);
		idx = strtok_r(NULL, " ", &saveptr);
		i = atoi(idx);

		if ( isAllowedNumber(num) )
		{
			flushPhoneSerial();
			Serial1.print(F("AT+CMGR="));
			Serial1.println(idx);
			readPhoneLine(atTimeout, false); // Discard the first line
			readPhoneLine(atTimeout, false);
			strncpy(msg, replybuffer, MAXBUFFER - 1);
			parseSMS(msg, num);
		}

		// Pat the watchdog
		wdt_reset();

		// Give time from the SMS management
		delay(2000);

		deleteOneSMS(i);

		Serial1.println(F("AT+CMGL=\"ALL\""));
		readPhoneLine(atTimeout, false);
	}
}

void parseSMS(const char *msg, const char *num)
{
	char parsedMsg[MAXBUFFER];
	char *cmd;
	char *saveptr;

	// Parse the message
	strncpy(parsedMsg, msg, MAXBUFFER - 1);
	cmd = strtok_r(parsedMsg, "#", &saveptr);

	// Check if command or programming SMS
	if ( strcasecmp(cmd, "TRACK") == 0 )
	{
		// TRACK function: TRACK#ON#numsec
		// This sends a message every <numsec>
		cmd = strtok_r(NULL, "#", &saveptr);
		if ( strcasecmp(cmd, "ON") == 0 )
		{
			trackOn = true;
			sleepMode = false;

			cmd = strtok_r(NULL, "#", &saveptr);
			trackSec = atoi(cmd);
			strncpy(trackNum, num, MAXTELDIGIT - 1);

			// The minimum pause between tracking will be 10 seconds
			if ( trackSec < 10 ) trackSec = 10;

			trackLast = millis();
#ifdef DEBUG
			if ( debug >= 1 )
			{
				Serial.print(F("--> Track function: ON  Pause: "));
				Serial.print(trackSec);
				Serial.print(F(" Num: "));
				Serial.println(trackNum);
			}
#endif
		}
		else if ( strcasecmp(cmd, "OFF") == 0 )
		{
			trackOn = false;

			if ( strcasecmp(sleepMode_conf,"ON") == 0 )
			{
				sleepMode = true;
			}
			else
			{
				sleepMode = false;
			}
		}
	}
	else if ( strcasecmp(cmd, "WIRELESS") == 0 )
	{
		// WIRELESS function: WIRELESS#
		// This sends a list of wireless devices in the area

		// REMOVED FOR XADOW
	}
	else if ( strcasecmp(cmd, "GETPOS") == 0 )
	{
		// Send current position via SMS
		if ( isGPSOn == false )
		{
			turnGPSOn();
		}
		resendSMS = true;
	}
	else if ( strcasecmp(cmd, "GETPOSGSM") == 0 )
	{
		char mcc[MAXBUFFER];
		char mnc[MAXBUFFER];
		char cellid[MAXBUFFER];
		char lac[MAXBUFFER];

		// Send current GSM cell seen by the tracker
		getATReply(F("AT+CENG?"), atTimeout);
		readPhoneLine(atTimeout, false);
		if ( parseGSMPos(replybuffer,mcc,mnc,cellid,lac) == 0 )
		{
			sendGsmPosSMS(num, mcc, mnc, cellid, lac);
		}
	}
	else if ( strcasecmp(cmd, "INFO") == 0 )
	{
		// INFO function: INFO# sends debug info
		sendSMS(num, "hereyouARE - $Revision: 2.0 $ Xadow", true);
	}
	else
#ifdef UBIDOTS
	if ( strcasecmp(cmd, "UBIDOTS") == 0 )
	{
		// TRACK function: UBIDOTS#ON
		cmd = strtok_r(NULL, "#", &saveptr);
		if ( strcasecmp(cmd, "ON") == 0 )
		{
			ubiOn = true;
			sleepMode = false;
		}
		else
		{
			ubiOn = false;

			if ( strcasecmp(sleepMode_conf,"ON") == 0 )
			{
				sleepMode = true;
			}
			else
			{
				sleepMode = false;
			}

		}
	} else
#endif
	{
		// If this is not a command, it probably is a programming message
		// Try to parse it and save configuration
		if ( parseConf(msg) == 1 )
		{
			// Parameter updated
			writeConf();
			sendSMS(num, "Configuration updated", false);
		}
		else
		{
			// No parameters updated
			sendSMS(num, "Configuration not updated. Check syntax.", false);
		}
	}
}

void deleteOneSMS(int idx)
{
	char tmpBuff[15];

	// Delete the SMS indexed as "idx"
	sprintf(tmpBuff, "AT+CMGD=%d", idx);

	while ( getATReplyCheck(tmpBuff, F("OK"), atTimeout) == false ) ;

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> SMS deleted"));
	}
#endif
}

boolean isAllowedNumber(const char* tel)
{
	int i;
	boolean allowed = false;

	// If the number is the one configured, or if the configured number is still the factory
	// default, allow operations
	if ( strstr(tel, primaryNumber_conf) != NULL || strcmp(primaryNumber_conf, "+1234567890") == 0 )
	{
		allowed = true;
	}
	else
	{
		for ( i = 0; i < MAXALLOWED; i++ )
		{
			if ( strstr(tel, allowedNumbers_conf[i]) != NULL )
			{
				allowed = true;
				break;
			}
		}
	}

	return allowed;
}

int parseGSMPos(char* CENGstr, char* gsmMcc, char* gsmMnc, char* gsmCellid, char* gsmLac )
{

	/*
	 *
	 * Parse the output of AT+CENG parameter to extract cell info
	 *
	 * Needed parameters are MCC, MNC, CELLID and LAC
	 * Format is:
	 *
	 * <CELL>,"<ARFCN>,<RXL>,<RXQ>,<MCC>,<MCN>,<BSIC>,<CELLID>,<LAC>,<RLA>,<TXP>,<TA>"
	 *
	 *
	 */
	char *skip;
	char *mcc;
	char *mnc;
	char *cellid;
	char *lac;

	if ( strstr(CENGstr,"0,\"") != NULL )
	{
		//
		// Parse response
		//

		// Skip some fields
		for (int i=0; i < 4; i++ )
		{
			skip = strsep(&CENGstr, ",");
		}

		// Get MCC
		mcc = strsep(&CENGstr, ",");
		if ( !mcc ) return(-1);
		strncpy(gsmMcc,mcc,MAXBUFFER - 1);

		// Get MNC
		mnc = strsep(&CENGstr, ",");
		if ( !mnc ) return(-1);
		strncpy(gsmMnc,mnc,MAXBUFFER - 1);

		// Skip a field
		skip = strsep(&CENGstr, ",");
		if ( !skip ) return(-1);

		// Get CELLID
		cellid = strsep(&CENGstr, ",");
		if ( !cellid ) return(-1);
		strncpy(gsmCellid,cellid,MAXBUFFER - 1);

		// Get LAC
		lac = strsep(&CENGstr, ",");
		if ( !lac ) return(-1);
		strncpy(gsmLac,lac,MAXBUFFER - 1);

#ifdef DEBUG
		if ( debug >= 1 )
		{
			Serial.print(F("--> GSM MCC: "));
			sprintf(buff, "%s", gsmMcc);
			Serial.println(buff);
			Serial.print(F("--> GSM MNC: "));
			sprintf(buff, "%s", gsmMnc);
			Serial.println(buff);
			Serial.print(F("--> GSM CELLID: "));
			sprintf(buff, "%s", gsmCellid);
			Serial.println(buff);
			Serial.print(F("--> GSM LAC: "));
			sprintf(buff, "%s", gsmLac);
			Serial.println(buff);
		}
#endif
	}

	return(0);
}

int checkSafeArea()
{
	if ( safeAreaWarnSent == 0 )
	{
		// Compare coordinates, but only with 4 decimal instead of 6 to avoid false positive
		if ( (double)round(currentLat * 10000) / 10000 < (double)round(min(safeAreaLat1, safeAreaLat2) * 10000) / 10000 ||
		     (double)round(currentLat * 10000) / 10000 > (double)round(max(safeAreaLat1, safeAreaLat2) * 10000) / 10000 ||
		     (double)round(currentLong * 10000) / 10000 < (double)round(min(safeAreaLong1, safeAreaLong2) * 10000) / 10000 ||
		     (double)round(currentLong * 10000) / 10000 > (double)round(max(safeAreaLong1, safeAreaLong2) * 10000) / 10000 )
		{
			safeAreaWarnSent = 1;
			sendSMS(primaryNumber_conf, "WARNING: Safe Area Left", false);

#ifdef DEBUG
			if ( debug >= 1 )
			{
				Serial.println(F("--> Safe Area SMS sent"));
			}
#endif
		}
	}
	else
	{
		// Check if it's back in safa area
		// Compare coordinates, but only with 4 decimal instead of 6 to avoid false positive
		if ( (double)round(currentLat * 10000) / 10000 >= (double)round(min(safeAreaLat1, safeAreaLat2) * 10000) / 10000 &&
		     (double)round(currentLat * 10000) / 10000 <= (double)round(max(safeAreaLat1, safeAreaLat2) * 10000) / 10000 &&
		     (double)round(currentLong * 10000) / 10000 >= (double)round(min(safeAreaLong1, safeAreaLong2) * 10000) / 10000 &&
		     (double)round(currentLong * 10000) / 10000 <= (double)round(max(safeAreaLong1, safeAreaLong2) * 10000) / 10000 )
		{
			safeAreaWarnSent = 0;
			sendSMS(primaryNumber_conf, "WARNING: I'm back to safe area", false);

#ifdef DEBUG
			// Print the calling number
			if ( debug >= 1 )
			{
				Serial.println(F("--> Safe Area SMS sent"));
			}
#endif
		}

	}
#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> Safe Area check"));
		Serial.print(F("----> "));
		Serial.println(currentLat, 6);
		Serial.print(F("----> "));
		Serial.println(currentLong, 6);
		Serial.print(F("------> "));
		Serial.println(safeAreaLat1, 6);
		Serial.print(F("------> "));
		Serial.println(safeAreaLong1, 6);
		Serial.print(F("------> "));
		Serial.println(safeAreaLat2, 6);
		Serial.print(F("------> "));
		Serial.println(safeAreaLong2, 6);
	}
#endif

}

/*************************************************************************/
/* GPS Functions section                                                 */
/*************************************************************************/

void getGPS(void)
{
	replybuffer[0] = '\0';

	getATReply(F("AT+CGNSINF"), atTimeout);

	// Copy the string skipping the CGNSINF
	strncpy(gpsbuff, replybuffer, MAXGPSBUFFER - 1);

//#ifdef DEBUG     FIXME
	if ( debug >= 3 )
	{
		Serial.print(F("--> GPS Sentence: "));
		Serial.println(gpsbuff);
	}
//#endif
}

int parseCGNSINF(char* CGNSINFstr, char* coordinates, char* gpsAlt, int *satNum, unsigned long *gpsDate, char* gpsSpeed)
{

	/*
	 *
	 * Parse the GPS sentence returned by the GPS module
	 *
	 * 0   GPS run status
	 * 1   Fix status
	 * 2   UTC date & Time yyyyMMddhh yyyy: [1980,2039]
	 *                          mmss.sss MM : [1,12]
	 *                          dd: [1,31]
	 *                          hh: [0,23]
	 *                          mm: [0,59]
	 *                          ss.sss:[0.000,60.999]
	 * 3   Latitude ±dd.dddddd [-90.000000,90.000000]
	 * 4   Longitude ±ddd.dddddd [-180.000000,180.000000]
	 * 5   MSL Altitude meters
	 * 6   Speed Over Ground Km/hour [0,999.99]
	 * 7   Course Over Ground degrees [0,360.00]
	 * 8   Fix Mode -- 0,1,2[1]
	 * 9   Reserved1
	 * 10  HDOP -- [0,99.9]
	 * 11  PDOP -- [0,99.9]
	 * 12  VDOP -- [0,99.9]
	 * 13  Reserved2
	 * 14  GPS Satellites in View -- [0,99]
	 * 15  GNSS Satellites Used -- [0,99]
	 * 16  GLONASS Satellites in View -- [0,99]
	 * 17  Reserved3 19 C/N0 max
	 * 18  HPA
	 * 19  VPA
	 *
	 */
	int gpsFix = 0;

	char *skip;
	char *fix;
	char *date;
	char *lat, *lon;
	char *alt;
	char *spd;
	char *sat;
	char tmpDate[12];

	if ( strstr(CGNSINFstr,"+CGNSINF:") != NULL )
	{
		//
		// Parse GPS response
		//

		// GPS Run status (not used)
		skip = strsep(&CGNSINFstr, ",");
		if ( !skip ) return(-1);

		// Fix status
		fix = strsep(&CGNSINFstr, ",");
		if ( !fix ) return(-1);
		gpsFix = atoi(fix);

		// Date & time
		date = strsep(&CGNSINFstr, ",");
		if ( !date ) return(-1);
		strncpy(tmpDate,date + 2,10);
		*gpsDate = atol(tmpDate);

		// Lat and Long
		lat = strsep(&CGNSINFstr, ",");
		if ( !lat ) return(-1);
		lon = strsep(&CGNSINFstr, ",");
		if ( !lon ) return(-1);
		strncpy(coordinates,lat,11);
		strncat(coordinates,",",1);
		strncat(coordinates,lon,11);

		// Altitude
		alt = strsep(&CGNSINFstr, ",");
		if ( !alt ) return(-1);
		strncpy(gpsAlt, alt, MAXBUFFER - 1);

		// Speed
		spd = strsep(&CGNSINFstr, ",");
		if ( !spd ) return(-1);
		strncpy(gpsSpeed, spd, MAXBUFFER - 1);

		// Skip some fields
		for (int i=0; i < 8; i++ )
		{
			skip = strsep(&CGNSINFstr, ",");
		}

		// Number of satellites used
		sat = strsep(&CGNSINFstr, ",");
		if ( !sat ) return(-1);
		*satNum = atoi(sat);

		// FIXME
//#ifdef DEBUG
		if ( debug >= 1 )
		{
			Serial.print(F("--> Fix Status: "));
			sprintf(buff, "%d", gpsFix);
			Serial.println(buff);
			Serial.print(F("--> Date and time: "));
			sprintf(buff, "%lu", *gpsDate);
			Serial.println(buff);
			Serial.print(F("--> Coordinates: "));
			Serial.println(coordinates);
			Serial.print(F("--> Altitude: "));
			Serial.println(gpsAlt);
			Serial.print(F("--> Speed: "));
			Serial.println(gpsSpeed);
			Serial.print(F("--> Num of Satellites: "));
			sprintf(buff, "%d", *satNum);
			Serial.println(buff);
		}
//#endif
	}

	// Returns the fix
	return(gpsFix);
}

/*************************************************************************/
/* EEPROM STORAGE                                                       */
/*************************************************************************/

int writeEEPROM(char *buffer, int addr)
{
	int i = 0;
	int len = strlen(buffer);

	while (i < len)
	{
		EEPROM.write(addr, buffer[i]);
		addr++;
		i++;
		delay(10);

#ifdef DEBUG
		if ( debug >= 2 )
		{
			Serial.println(F("--> EEPROM byte written"));
		}
#endif

	}
	EEPROM.write(addr, '\0');
	delay(10); //add a small delay

#ifdef DEBUG
	if ( debug >= 1 )
	{
		Serial.println(F("--> EEPROM written"));
	}
#endif

	return (i);
}

void EEPROMWritelong(int address, long value)
{
	//Decomposition from a long to 4 bytes by using bitshift.
	//One = Most significant -> Four = Least significant byte
	byte four = (value & 0xFF);
	byte three = ((value >> 8) & 0xFF);
	byte two = ((value >> 16) & 0xFF);
	byte one = ((value >> 24) & 0xFF);

	//Write the 4 bytes into the eeprom memory.
	EEPROM.write(address, four);
	EEPROM.write(address + 1, three);
	EEPROM.write(address + 2, two);
	EEPROM.write(address + 3, one);
}

long EEPROMReadlong(long address)
{
	//Read the 4 bytes from the eeprom memory.
	long four = EEPROM.read(address);
	long three = EEPROM.read(address + 1);
	long two = EEPROM.read(address + 2);
	long one = EEPROM.read(address + 3);

	//Return the recomposed long by using bitshift.
	return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

/*************************************************************************/
/* Files management (flash) Functions section                   */
/*************************************************************************/

int loadConf()
{
	int ret = 0;
	int x = 0;
	int y = 0;
	int addr = 0;

	char confBuffer[MAXBUFFER];

	// Check if conf exists, otherwise initialize it
	if ( EEPROMReadlong(MAGICADDR) != magic )
	{
		writeConf();
	}


	// Read first char from eeprom
	char b = EEPROM.read(addr); // Read the first byte

	// Load the complete file in an array
	while ( b != 0xFF && addr < MAXEEPROM )
	{
		if ( y < MAXBUFFER - 1 && x <= MAXCONFLINES )
		{
			confBuffer[y] = b;
			if ( confBuffer[y] == '\n' )
			{
				confBuffer[y] = '\0';
				y = 0;

				parseConf(confBuffer);
				x++;
			}
			else
			{
				y++;
			}
		}
		else
		{
			if ( x < MAXCONFLINES )
			{
				confBuffer[y] = '\0';
				y = 0;

				parseConf(confBuffer);
			}
		}

		addr++;
		b = EEPROM.read(addr);
	}

	return (ret);
}

int parseConf(const char *confBuffer)
{
	/*
	 * Format: XX|YYYYYYYYYY
	 *
	 *      X = 2 chars label
	 *      Y variable content closed by newline
	 *
	 * NOTE: When you add a new parameter, remember to update writeConf() function
	 *
	 * NOTE: pay attention to dependency sleepMode/safeArea
	 *
	 * Returns 1 if at least a parameter has been updated
	 *
	 */
	int ret = 0;
	char opt[3];
	char *saveptr1, *saveptr2;
	char *token, *subtoken;
	char tmpSafeArea[MAXBUFFER];

	// Check string format for consistency
	if ( confBuffer[2] != '|' && confBuffer[2] != '#' )
	{
		// Something is wrong in the parameter
		return(ret);
	}

	strncpy(opt, confBuffer, 2);
	opt[2] = '\0';

	// Load the variables with the proper values
	if ( strcasecmp(opt, "PN") == 0 )
	{
		strncpy(primaryNumber_conf, confBuffer + 3, MAXTELDIGIT - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "A1") == 0 )
	{
		strncpy(allowedNumbers_conf[0], confBuffer + 3, MAXTELDIGIT - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "A2") == 0 )
	{
		strncpy(allowedNumbers_conf[1], confBuffer + 3, MAXTELDIGIT - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "A3") == 0 )
	{
		strncpy(allowedNumbers_conf[2], confBuffer + 3, MAXTELDIGIT - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "A4") == 0 )
	{
		strncpy(allowedNumbers_conf[3], confBuffer + 3, MAXTELDIGIT - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "SE") == 0 )
	{
		strncpy(searchEng_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "SA") == 0 )
	{
		strncpy(safeArea_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;

		// Parse safe area
		strncpy(tmpSafeArea, safeArea_conf, MAXBUFFER - 1);
		token = strtok_r(tmpSafeArea, "#", &saveptr1);
		subtoken = strtok_r(token, ";", &saveptr2);
		safeAreaLat1 = atof(subtoken);
		subtoken = strtok_r(NULL, ";", &saveptr2);
		safeAreaLong1 = atof(subtoken);
		token = strtok_r(NULL, "#", &saveptr1);
		subtoken = strtok_r(token, ";", &saveptr2);
		safeAreaLat2 = atof(subtoken);
		subtoken = strtok_r(NULL, ";", &saveptr2);
		safeAreaLong2 = atof(subtoken);
	}
	if ( strcasecmp(opt, "SF") == 0 )
	{
		strncpy(safeOnOf_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;

		// Sets the "static" variable
		if ( strcasecmp(safeOnOf_conf, "ON") == 0 )
		{
			safeAreaOn = true;

			// Force the sleepMode to OFF
			parseConf("SM#OFF");
		}
		else
		{
			safeAreaOn = false;
		}
	}
	if ( strcasecmp(opt, "SM") == 0 )
	{
		strncpy(sleepMode_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;

		// Sets the "static" variable
		if ( strcasecmp(sleepMode_conf, "ON") == 0 )
		{
			sleepMode = true;

			// Force safe area to OFF
			parseConf("SF#OFF");
		}
		else
		{
			sleepMode = false;
		}
	}
	if ( strcasecmp(opt, "MU") == 0 )
	{
		strncpy(measureUnit_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;
	}
#ifdef UBIDOTS
	if ( strcasecmp(opt, "UT") == 0 )
	{
		strncpy(ubiToken_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;
	}
	if ( strcasecmp(opt, "UI") == 0 )
	{
		strncpy(ubiIdVariable_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;
	}

	if ( strcasecmp(opt, "UA") == 0 )
	{
		strncpy(ubiAPN_conf, confBuffer + 3, MAXBUFFER - 1);
		ret = 1;
	}
#endif

#ifdef DEBUG
	if ( debug >= 3 )
	{
		Serial.print(F("--> Red variable: "));
		Serial.println(confBuffer);
	}
#endif

#ifdef DEBUG
	if ( debug >= 4 )
	{
		Serial.print(F("--> Loaded variable: "));
		Serial.println(primaryNumber_conf);
		Serial.println(allowedNumbers_conf[0]);
		Serial.println(allowedNumbers_conf[1]);
		Serial.println(allowedNumbers_conf[2]);
		Serial.println(allowedNumbers_conf[3]);
		Serial.println(searchEng_conf);
		Serial.println(safeArea_conf);
		Serial.println(safeOnOf_conf);
		Serial.println(sleepMode_conf);
		Serial.println(measureUnit_conf);
		Serial.println(ubiToken_conf);
		Serial.println(ubiIdVariable_conf);
		Serial.println(ubiAPN_conf);
	}
#endif

	return (ret);
}

int writeConf()
{
	int ret = 0;
	int addr = 0;
	int writtenByte = 0;

	// Disable watchdog
	wdt_disable();

	// Initialize the EEPROM: write a 0 to MAXEEPROM-1 bytes of the EEPROM
	for (int i = 0; i < MAXEEPROM - 1; i++)
	{
		EEPROM.write(i, 0xFF);
	}

	// Write all the variables
	writtenByte = writeEEPROM("PN|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(primaryNumber_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("A1|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(allowedNumbers_conf[0], addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("A2|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(allowedNumbers_conf[1], addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("A3|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(allowedNumbers_conf[2], addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("A4|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(allowedNumbers_conf[3], addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("SE|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(searchEng_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("TZ|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("SA|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(safeArea_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("SF|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(safeOnOf_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("SM|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(sleepMode_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("MU|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(measureUnit_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;

#ifdef UBIDOTS
	writtenByte = writeEEPROM("UT|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(ubiToken_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("UI|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(ubiIdVariable_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("UA|", addr);
	addr += writtenByte;
	writtenByte = writeEEPROM(ubiAPN_conf, addr);
	addr += writtenByte;
	writtenByte = writeEEPROM("\n", addr);
	addr += writtenByte;
#endif

	// Write MAGIC number for conf check
	EEPROMWritelong(MAGICADDR,magic);

#ifdef DEBUG
	if ( debug >= 2 )
	{
		Serial.print(F("--> Config file written"));
	}
#endif

	// Enable watchdog
	wdt_enable(WDTO_8S);

	return (ret);
}

// Check when the last battery warn was sent
boolean lastWarningToday(unsigned long gpsDate)
{
	boolean ret = false;

	if ( EEPROMReadlong(LASTWARNADDR) != gpsDate )
	{
		EEPROMWritelong(LASTWARNADDR,gpsDate);
		ret = false;
	}
	else
	{
		ret = true;
	}

	return(ret);
}

/*************************************************************************/
/* Phone Functions                                                       */
/*************************************************************************/

void getBatteryState(int *charge, int *batteryLevel)
{
	char *saveptr1, *saveptr2;
	int fstChrg = 0;
	int fstLevel = 0;

	// Query the battery status, expected result:
	// +CBC: 0,56,3848
	//            status,%,voltage
	//            status 0    not charging
	//            status 1    charging
	//            status 2    charged

	getATReply(F("AT+CBC"), atTimeout);
	if ( strtok_r(replybuffer, ":", &saveptr1) )
	{
		fstChrg = atoi(strtok_r(strtok_r(NULL, ":", &saveptr1), ",", &saveptr2));
		fstLevel = atoi(strtok_r(NULL, ",", &saveptr2));

		// If the reads match, return the correct values
		*charge = fstChrg;
		*batteryLevel = fstLevel;
	}
}

boolean turnGSMOn()
{
	// Turn on GSM
	pinMode(A5, OUTPUT);

	while ( getATReplyCheck(F("AT"), F("OK"), atTimeout) == false )
	{
		digitalWrite(A5,HIGH);
		delay(500);
		digitalWrite(A5,LOW);
		delay(1000);
		digitalWrite(A5,HIGH);
		delay(3000);

		// Try to disable phone local echo (if on)
		getATReply(F("ATE0"), atTimeout);
	}

	// Set some options
	getATReply(F("AT+IPR=9600"), atTimeout); // Baud
	getATReply(F("AT+ECHARGE=1"), atTimeout); // Enable battery charging
	getATReply(F("AT+CENG=1"), atTimeout); // Engineer options ON

#ifdef DEBUG
	if ( debug >= 2 )
	{
		Serial.println(F("--> GSM turned ON"));
	}
#endif
}

boolean turnGPSOn()
{
	char count = 0;

	// Pat the watchdog
	wdt_reset();

	// Turn on GPS
	while ( getATReplyCheck(F("AT+CGNSPWR=1"), F("OK"), atTimeout) == false && count < 5 )
	{
		count++;
		delay(100);
	}

	if ( count < 5 )
	{
		isGPSOn = true;
		gpsOnTime = millis();

		delay(1000);

		// Check if ON
		while ( getATReplyCheck(F("AT+CGNSSEQ=GGA"), F("ERROR"), atTimeout) == true )
		{
			getATReply(F("AT+CGNSPWR=0"), atTimeout);
			delay(1000);
			getATReply(F("AT+CGNSPWR=1"), atTimeout);
			delay(1000);
		}
	}

#ifdef DEBUG
	if ( debug >= 2 )
	{
		Serial.println(F("--> GPS turned ON"));
	}
#endif
}

boolean turnGPSOff()
{
	char count = 0;

	// Turn off GPS
	while ( getATReplyCheck(F("AT+CGNSPWR=0"), F("OK"), atTimeout) == false && count < 5 )
	{
		count++;
		delay(100);
	}

	if ( count < 5 )
	{
		isGPSOn = false;
	}

#ifdef DEBUG
	if ( debug >= 2 )
	{
		Serial.println(F("--> GPS turned OFF"));
	}
#endif
}

int readPhoneLine(int timeout, boolean multiline)
{
	int idx = 0;

	while ( timeout-- )
	{
		if ( idx >= MAXATBUFFER - 1 )
		{
			break;
		}

		while ( Serial1.available() )
		{
			char c =  Serial1.read();
			if ( c == '\r' ) continue;
			if ( c == 0xA )
			{
				// the first 0x0A is ignored
				if ( idx == 0 )
					continue;

				if ( !multiline )
				{
					timeout = 0; // the second 0x0A is the end of the line
					break;
				}
			}
			replybuffer[idx] = c;
			idx++;
		}

		if ( timeout == 0 )
		{
			break;
		}
		delay(1);
	}
	replybuffer[idx] = '\0';
	return idx;
}

//
// getATReplyCheck
// Version for FLASH stored parameters
//
boolean getATReplyCheck(const __FlashStringHelper *send, const __FlashStringHelper *reply, int timeout)
{
	getATReply(send, timeout);

	if ( strncmp_P(replybuffer, (const char *) reply, MAXATBUFFER - 1) == 0 )
	{
		return (true);
	}
	else {
		return (false);
	}
}

//
// getATReplyCheck
// Version with variable
//
boolean getATReplyCheck(char *send, const __FlashStringHelper *reply, int timeout)
{
	getATReply(send, timeout);

	if ( strncmp_P(replybuffer, (const char *) reply, MAXATBUFFER - 1) == 0 )
	{
		return (true);
	}
	else {
		return (false);
	}
}

//
// getATReply
// Version for FLASH stored parameters
//
void getATReply(const __FlashStringHelper *send, int timeout)
{
	flushPhoneSerial();

	//#ifdef DEBUG   FIXME
	if ( debug >= 2 )
	{
		Serial.print(F("--> Sending AT command: "));
		Serial.println(send);
	}
	//#endif

	Serial1.println(send);

	readPhoneLine(timeout, false);
	//#ifdef DEBUG    FIXME
	if ( debug >= 2 )
	{
		Serial.print(F("--> Receiving AT input: "));
		Serial.println(replybuffer);
	}
	//#endif
}

//
// getATReply
// Version with variable
//
void getATReply(char *send, int timeout)
{
	flushPhoneSerial();

	//#ifdef DEBUG   FIXME
	if ( debug >= 2 )
	{
		Serial.print(F("--> Sending AT command: "));
		Serial.println(send);
	}
	//#endif

	Serial1.println(send);

	readPhoneLine(timeout, false);
	//#ifdef DEBUG    FIXME
	if ( debug >= 2 )
	{
		Serial.print(F("--> Receiving AT input: "));
		Serial.println(replybuffer);
	}
	//#endif
}

// Flush serial data
void flushPhoneSerial()
{
	// Read all to discard data
	int loop = 0;
	while ( loop++ < 40 )
	{
		while ( Serial1.available() )
		{
			Serial1.read();
			// Reset timer
			loop = 0;
		}
		delay(1);
	}
}

#ifdef UBIDOTS
/*************************************************************************/
/* Ubidots Functions                                                       */
/*************************************************************************/

// This function is to send the sensor data to Ubidots, you should
// see the new value in Ubidots after executing this function
void ubidotsSaveValue(char *value)
{
	int num;
	char len[4];
	char var[MAXATBUFFER];
	char coord[MAXBUFFER];
	char *saveptr;

	strncpy(coord, value, MAXBUFFER - 1);
	strcpy(var, "{\"value\": 10,\"context\":{\"lat\":");
	strncat(var, strtok_r(coord, ",", &saveptr), 9);
	strcat(var, ",\"lng\":");
	strncat(var, strtok_r(NULL, ",", &saveptr), 9);
	strcat(var, "}}");

	num = strlen(var);
	sprintf(len, "%d", num);

	getATReply(F("AT+CGATT?"), atTimeout);

	// Replace with your providers' APN
	Serial1.print(F("AT+CSTT="));
	Serial1.print(ubiAPN_conf);
	Serial1.println();
	delay(1000);

	Serial1.print(F("AT+CSTT?"));
	Serial1.println();
	delay(1000);

	// Pat the watchdog
	wdt_reset();

	// Bring up wireless connection
	getATReply(F("AT+CIICR"), atTimeout);
	delay(3000);

	// Pat the watchdog
	wdt_reset();

	// Get local IP address
	getATReply(F("AT+CIFSR"), atTimeout);
	getATReply(F("AT+CIPSPRT=0"), atTimeout);
	delay(3000);
	// Start up the connection
	getATReply(F("AT+CIPSTART=\"tcp\",\"things.ubidots.com\",\"80\""), atTimeout);
	delay(3000);

	// Pat the watchdog
	wdt_reset();

	// Wait a bit more to allow data connection to set
	delay(6000);

	// Pat the watchdog
	wdt_reset();

	// Begin to send data to remote server
	Serial1.println(F("AT+CIPSEND"));
	delay(3000);

	Serial1.print(F("POST /api/v1.6/variables/"));
	Serial1.print(ubiIdVariable_conf);
	delay(100);

	Serial1.println(F("/values HTTP/1.1"));
	delay(100);

	Serial1.println(F("Content-Type: application/json"));
	delay(100);

	Serial1.print(F("Content-Length: "));
	Serial1.println(len);
	delay(100);

	Serial1.print(F("X-Auth-Token: "));
	delay(100);
	Serial1.println(ubiToken_conf);
	delay(100);
	Serial1.println(F("Host: things.ubidots.com"));
	delay(100);
	Serial1.println();
	delay(100);
	Serial1.println(var);
	delay(100);
	Serial1.println();
	delay(100);
	Serial1.println((char)26);

	// Pat the watchdog
	wdt_reset();

	delay(7000);
	Serial1.println();

	// Pat the watchdog
	wdt_reset();

	// Close the connection
	getATReply(F("AT+CIPCLOSE"), atTimeout);
	delay(1000);

	// Turn off wireless
	getATReply(F("AT+CIPSHUT"), atTimeout);
	delay(1000);
}
#endif

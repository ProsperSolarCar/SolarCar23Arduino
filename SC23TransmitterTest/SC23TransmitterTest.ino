#include <SPI.h>
#include <RH_RF95.h>
#include <string.h>


//=============================================================================
// General Config

// Keys for the different data fields.  These keys must match what the display
//  program on the Android tablets is expecting.
#define KEY_SPEED         "spd"
#define KEY_MAIN_VOLTAGE  "mn"
#define KEY_AUX_VOLTAGE   "aux"
#define KEY_SOLAR_CURRENT "sol"
#define KEY_MOTOR_CURRENT "mtr"

// NOTE:  Each transmitted item adds up to 11 bytes to the packet, with the key,
//  punctuation/separators, and a floating point fAuxVoltageue with 3 digits to the left
//  and 2 to the right of the period.
//  "KEY:000.00,"
// Make sure the radiopacket is large enough to contain the characters.

int iLoopCounter;
int iRadioLoopFreq = 2; // every iRadioLoopFreq loops, send a radio packet

//=============================================================================
// Radio Config

// Solar Car Transmitter hardware wiring
#define RFM95_EN 10
#define RFM95_INT 2 // G0
#define RFM95_SCK 13
#define RFM95_MISO 12
#define RFM95_MOSI 11
#define RFM95_CS 4
#define RFM95_RST 9

// Feather 32u4:
//#define RFM95_CS   8
//#define RFM95_RST  4
//#define RFM95_INT  7

// Must match RX's freq!
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int iRadioIsLive = 0;

#define MAX_RADIO_PACKET_LENGTH 151
char radiopacket[MAX_RADIO_PACKET_LENGTH] = "";
String strRadioOutput;

float  fTime = 0.25f;
float  fTimeDelta = 0.1f;


//=============================================================================
// Sensor Config
// Sensor Pins
// A1, A2, A3, A5

// telemetry
float fAuxVoltage;
float fSolarCurrent;
float fMotorCurrent;
float fMainVoltage;
float fSpeedMph;

String strMainVoltage;
String strAuxVoltage;
String strSolarCurrent;
String strMotorCurrent;
String strSpeedMph;




//-----------------------------------------------------------------------------
void setupRadio ()
  {
  Serial.print("status:Arduino LoRa TX Test!,\n");

  //pinMode (RFM95_EN, OUTPUT);
  //pinMode (RFM95_INT, INPUT);
  //pinMode (RFM95_SCK, OUTPUT);
  //pinMode (RFM95_MISO, INPUT);
  //pinMode (RFM95_MOSI, OUTPUT);

  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  iRadioIsLive = 1;
  while (!rf95.init())
    {
    Serial.print("status:LoRa radio init failed,\n");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    //while (1);
    iRadioIsLive = 0;
    break;
    }
  delay (100); // must delay or Arduino hangs.
  if (!iRadioIsLive)
    {
    Serial.print("status:LoRa radio failed.  Continuing without radio!,\n");
    return;
    }
  Serial.print("status:LoRa radio init OK!,\n");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ))
    {
    Serial.print("status:setFrequency failed.  Stopping program,");
    while (1); // stops program
    }
  Serial.print("status:Set Freq to: "); Serial.print(RF95_FREQ); Serial.print(",");

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
  }

//-----------------------------------------------------------------------------
void setupSensors ()
  {
  // NOTE: Sensors read on Analog pins 1,2,3 and 5.  Analog pins do not need pinMode set.

  }

//-----------------------------------------------------------------------------
void setupSerial ()
  {
  Serial.begin(9600); // starts the data cranking out of the UART to the BT transmitter
  //while (!Serial) delay(1); // Don't block on serial if you are running headless.
  delay (1000);
  }

//-----------------------------------------------------------------------------
void setup ()
  {
  iLoopCounter = 0;

  setupSerial ();
  Serial.print("status:Setup begin,\n");

  setupRadio();

  delay(100);
  Serial.print("status:Setup complete,\n");
  }



//-----------------------------------------------------------------------------
void updateSensors ()
  {
  fTime += fTimeDelta;
  if (fTime < 0.2f) fTimeDelta = 0.1f;
  if (fTime > 0.8f) fTimeDelta = -0.1f;

  fAuxVoltage   = fTime * 20.0f; // max  20
  fSolarCurrent = fTime * 20.0f; // max 20
  fMotorCurrent = fTime * 100.0f; // max 100
  fMainVoltage  = fTime * 90.0f; // max 90
  fSpeedMph     = fTime * 55.0f; // max 55

  strAuxVoltage   = String (fAuxVoltage, 2);
  strSolarCurrent = String (fSolarCurrent, 2);
  strMotorCurrent = String (fMotorCurrent, 2);
  strMainVoltage  = String (fMainVoltage, 2);
  strSpeedMph     = String (fSpeedMph, 2);
  }


//-----------------------------------------------------------------------------
void serialTransmitSensorOutput ()
  {
  String strSerialOutput = String(KEY_SPEED) + ":" + strSpeedMph     + "," +
                           KEY_MAIN_VOLTAGE  + ":" + strMainVoltage  + "," +
                           KEY_AUX_VOLTAGE   + ":" + strAuxVoltage   + "," +
                           KEY_SOLAR_CURRENT + ":" + strSolarCurrent + "," +
                           KEY_MOTOR_CURRENT + ":" + strMotorCurrent + ",";
  Serial.print (strSerialOutput);
  }

//-----------------------------------------------------------------------------
void radioTransmitSensorOutput ()
  {
  // If we don't have an initialized radio, there's nothing here to do.  Exit.
  if (! iRadioIsLive) return;

  strRadioOutput = String(KEY_SPEED) + ":" + strSpeedMph     + "," +
                   KEY_MAIN_VOLTAGE  + ":" + strMainVoltage  + "," +
                   KEY_AUX_VOLTAGE   + ":" + strAuxVoltage   + "," +
                   KEY_SOLAR_CURRENT + ":" + strSolarCurrent + "," +
                   KEY_MOTOR_CURRENT + ":" + strMotorCurrent + ",";

  const char * c = strRadioOutput.c_str();
  int dataLength = strRadioOutput.length();
  if (dataLength > MAX_RADIO_PACKET_LENGTH)
    {
    Serial.print("status:Radio Packet is too long to send.  Aborting send.,");
    return;
    }

  delay(10);
  sprintf(radiopacket, c);
  rf95.send((uint8_t *)radiopacket, dataLength);

  // Wait for packet to complete
  delay(10);
  rf95.waitPacketSent();
  }


//-----------------------------------------------------------------------------
void loop ()
  {
  delay(500); // Wait 0.5 second between updates, could also 'sleep' here!

  ++iLoopCounter;
  updateSensors ();
  //serialTransmitSensorOutput ();
  if ((iLoopCounter % iRadioLoopFreq) == 0)
    {
    // We update the sensors and display above more frequently than sending radio packets
    Serial.print("status:Sending Radio packet.,\n");
    radioTransmitSensorOutput ();
    iLoopCounter = 0;
    };
  }



#include <SPI.h>

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

float  fTime = 0.25f;
float  fTimeDelta = 0.1f;

//=============================================================================
// Sensor Config

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



// Pin 13 has an LED connected on most Arduino boards.
// give it a name:
int led = 13;

// the setup routine runs once when you press reset:
//-----------------------------------------------------------------------------
void setup ()
  {
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);


  Serial.begin(9600); // starts the data cranking out of the UART to the BT transmitter
  while (!Serial) delay(1);
  //Serial.println("Setup begin");
  //Serial.println("Setup complete");
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

// the loop routine runs over and over again forever:
//-----------------------------------------------------------------------------
void loop()
  {
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(500);               // wait for .5 second
  digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  delay(500);               // wait for .5 second

  updateSensors ();
  serialTransmitSensorOutput ();
  }

#include <SPI.h>
#include <RH_RF95.h>

//=============================================================================
// General Config

// Keys for the different data fields.  These keys must match what the display
//  program on the Android tablets is expecting.
#define KEY_SPEED         "spd"
#define KEY_MAIN_VOLTAGE  "mn"
#define KEY_AUX_VOLTAGE   "aux"
#define KEY_SOLAR_CURRENT "sol"
#define KEY_MOTOR_CURRENT "mtr"

#define KEY_STATUS        "status"
#define KEY_RSSI          "rssi"


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
//#define RFM95_EN 10
//#define RFM95_INT 2 // G0
//#define RFM95_SCK 13
//#define RFM95_MISO 12
//#define RFM95_MOSI 11
//#define RFM95_CS "not wired" // TODO: CS pin needs to be wired!!!!
//#define RFM95_RST 9

// Solar Car Receiver hardware wiring
#define RFM95_CS 4
#define RFM95_RST 2
#define RFM95_INT 3

// Feather 32u4:
//#define RFM95_CS   8
//#define RFM95_RST  4
//#define RFM95_INT  7

// Must match RX's freq!
#define RF95_FREQ 915.0

RH_RF95 rf95(RFM95_CS, RFM95_INT);
int     iRadioIsLive = 0;

int     iNewInputAvailable;
int16_t iRssi;
uint8_t szRadioInput[RH_RF95_MAX_MESSAGE_LEN];
uint8_t iRadioInputLength;




//-----------------------------------------------------------------------------
void setupRadio ()
  {
  //Serial.print("status:Arduino LoRa TX Test!,");

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
    Serial.print("status:LoRa radio init failed,");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    //while (1);
    iRadioIsLive = 0;
    break;
    }
  delay (100); // must delay or Arduino hangs.
  if (!iRadioIsLive)
    {
    Serial.print("status:LoRa radio failed.  Unable to continue!,");
    while (1);
    //return;
    }
  Serial.print("status:LoRa radio init OK!,");

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
void setupSerial ()
  {
  Serial.begin(9600); // starts the data cranking out of the UART to the BT transmitter
  while (!Serial) delay(1); // Don't block on serial if you are running headless.
  //delay (1000);
  }

//-----------------------------------------------------------------------------
void setup ()
  {
  pinMode(LED_BUILTIN, OUTPUT);
  iNewInputAvailable = 0;

  setupSerial ();
  Serial.print("status:Setup begin,");

  setupRadio();

  delay(100);
  Serial.print("status:Setup complete,");
  }


//-----------------------------------------------------------------------------
void serialTransmitSensorInput ()
  {
  if (! iNewInputAvailable) return;
  iNewInputAvailable = 0;

  if (iRadioInputLength == 0) return;

  Serial.print ((char*)szRadioInput);
  // Because input is a stream, we only want to output the rssi info
  //  when we are at the end of a packet.
  if (szRadioInput [iRadioInputLength - 1] == ',')
    {
    Serial.print (String(KEY_RSSI) + ":" + String (iRssi, DEC) + ",");
    };
  Serial.flush();
  }

//-----------------------------------------------------------------------------
void radioReceiveSensorInput ()
  {
  // If we don't have an initialized radio, there's nothing here to do.  Exit.
  if (! iRadioIsLive) return;

  if (rf95.available()) {
    // Should be a message for us now
    iRadioInputLength = sizeof(szRadioInput) - 1;

    digitalWrite(LED_BUILTIN, HIGH);
    if (rf95.recv(szRadioInput, &iRadioInputLength)) {

      if (iRadioInputLength > 0)
        {
        szRadioInput[iRadioInputLength] = '\0'; // Zero-terminate the message so we can print it.
        iRssi = rf95.lastRssi();
        iNewInputAvailable = 1;
        }
      }
    digitalWrite(LED_BUILTIN, LOW);
    }
  }

//-----------------------------------------------------------------------------
void loop ()
  {
  radioReceiveSensorInput ();
  serialTransmitSensorInput ();
  }

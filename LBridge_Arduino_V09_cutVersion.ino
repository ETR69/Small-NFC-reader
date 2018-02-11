/*
   LBridge scans the Freestyle Libre Sensor every 5 minutes
   and sends the current BG readings to the xDrip+ Android app.

   This sketch is based on the LimiTTer project from JoernL, the
   sample sketches for the BM019 module from Solutions Cubed and the
   protocol extensions done be savek-cc in the Wixel project.

   This code replaces the LimiTTer code without any changes needed.

   Hardwaresource in xDrip has to be set to "LimiTTer". Please use
   an xDrip+ version >= nightly build 2107/02/09

   The xBridge2 protocol is used to send BG readings to xDrip+. In case of failure it queues up not sended BG readings
   with the correct timestamp and send them within the next BLE connection. There should be no missed BG readings
   when the LimiTTer is worn. Battery performance is improved compared to LimiTTer.

   Wiring for UNO / Pro-Mini:

   Arduino          BM019           BLE-HM11
   IRQ: Pin 9       DIN: pin 2
   SS: pin 10       SS: pin 3
   MOSI: pin 11     MOSI: pin 5
   MISO: pin 12     MISO: pin4
   SCK: pin 13      SCK: pin 6
   I/O: pin 3 (VCC with Android 4)  VCC: pin 9
   I/O: pin 5                       TX:  pin 2
   I/O: pin 6                       RX:  pin 4
*/

/* 
 changes since 170716_1950:
  	- AT+RENEW fail mechanism disabled to ensure usage with modified fake modules
	- display serial status chars while sleeping
	- detect faulty sensorsa during normal lifetime of 14 days
	- removed fault in status display of spike removal
	- added AT command before AT+ commands to break BLE connection if active
	- hard system reset if AT+NOTI1 command cannot be processed
	- slightly modified debug oputput
  V09.01:
  - new version number / minor version number
*/

/* ************************************************ */
/* *** config #DEFINES **************************** */
/* ************************************************ */

#define N_RELEASE_VERSION

#define LB_DATETIME "180207_2208"
#define LB_VERSION "V09"        // version number
#define LB_MINOR_VERSION ".01"  // indicates minor version

#ifdef RELEASE_VERSION

#define LNAME    "LimiTTer"   // BLE name, must begin with "LimiTTer" to avoid misfunctions in xDrip+
// important features 
#define N_USE_DEAD_SENSOR     // we can test with a dead sensor
#define REMOVE_SPIKES         // use NFC data smoothing (LimiTTer method)?
#define AUTOCAL_WDT           // code for auto calibrating WDT timing
#define ATFAIL_REBOOT         // reboot system if AT+NOTI goes fail
#define N_ATRESET             // do a AT+RESET after every BLE wakeup?
#define TRANSFER_LIVETIME     // send sensor lifetime
#define N_XBEXT               // xbridge extended code

// less common settings for debug or test
#define N_DB_PKT              // display BG queue, 'X' command
#define N_DB_VOLTAGE            // display battery usage, 'V' command
#define N_SIMU_LIVETIME       // simulate sensor livetime
#define N_HW_BLE              // detect BLE conn status with system LED pin connected to arduino pin 2
#define N_DYNAMIC_TXID        // get TXID automatically
#define N_DB_NFC_DATA         // debug NFC data
#define N_SIMU_BG             // simulate BG readings from dead sensor, ramp curve
#define N_UPDATE_HM1X         // use with extreme care! update HM-11
#define N_DB_PROCESSING       // extended debug output
#define N_INIT_WITH_RENEW     // send AT+RENEW at BLE init - will kill fake modules as setting them to 115200
#define N_SAFETY_RENEW        // send AT+RENEW after 30 min of no BLE

#else /* RELEASE_VERSION */

#define LNAME    "LimiTTerTVW"  // BLE name, must begin with "LimiTTer" to avoid misfunctions in xDrip+
// important features 
#define USE_DEAD_SENSOR       // we can test with a dead sensor
#define REMOVE_SPIKES         // use NFC data smoothing (LimiTTer method)?
#define AUTOCAL_WDT           // code for auto calibrating WDT timing
#define ATFAIL_REBOOT         // reboot system if AT+NOTI goes fail
#define N_ATRESET             // do a AT+RESET after every BLE wakeup?
#define TRANSFER_LIVETIME     // send sensor lifetime
#define N_XBEXT               // xbridge extended code

// less common settings for debug or test
#define N_DB_PKT                // display BG queue, 'X' command
#define N_DB_VOLTAGE            // display battery usage, 'V' command
#define N_SIMU_LIVETIME       // simulate sensor livetime
#define N_HW_BLE              // detect BLE conn status with system LED pin connected to arduino pin 2
#define DYNAMIC_TXID        // get TXID automatically
#define N_DB_NFC_DATA         // debug NFC data
#define N_SIMU_BG               // simulate BG readings from dead sensor, ramp curve
#define N_UPDATE_HM1X         // use with extreme care! update HM-11
#define N_DB_PROCESSING         // extended debug output
#define N_INIT_WITH_RENEW     // send AT+RENEW at BLE init - will kill fake modules as setting them to 115200
#define N_SAFETY_RENEW        // send AT+RENEW after 30 min of no BLE

#endif /* RELEASE_VERSION */

#include <SPI.h>
#include <SoftwareSerial.h>
//#include <avr/sleep.h>
//#include <avr/power.h>
//#include <avr/wdt.h>
//#ifdef XBEXT
//#include "xbridge_libre.h"
//#endif

#define MAX_NFC_READTRIES 3   // amount of tries for every nfc block-scan
#define NFC_READ_RETRY 3      // amount of tries to recall read_memory() in case of NFC read error
#define STOP_SEND_READINGS (((14*24)+12)*60) // 14,5 days
#define SPIKE_RANGE 40        // eliminates spikes which are +- from the last reading

/* ********************************************* */
/* ********* LimiTTer stuff ******************** */
/* ********************************************* */

float calibv = 1.0;   // ratio of real clock with WDT clock

#define MIN_V 3450 // battery empty level
#define MAX_V 4050 // battery full level

#define WDT_8S_MASK 0b00100001
#define WDT_1S_MASK 0b00000110

const int SSPin = 10;   // Slave Select pin
const int IRQPin = 9;   // Sends wake-up pulse for BM019
const int NFCPin1 = 7;  // Power pin BM019
const int NFCPin2 = 8;  // Power pin BM019
const int NFCPin3 = 4;  // Power pin BM019
//const int BLEPin = 3;   // BLE power pin.
const int MOSIPin = 11;
const int SCKPin = 13;


byte RXBuffer[24];
byte NFCReady = 0;  // used to track NFC state
byte FirstRun = 1;

int sleepTime = 32;   // sleeptime in multipliers of 8 s

int noDiffCount = 0;
int sensorMinutesElapse;
float lastGlucose;
float trend[16];



/* ************************************************* */
/* *** control stuff ******************************* */
/* ************************************************* */

// global error counters
unsigned long loop_count = 0;          // of main loop
unsigned long normal_loop_count = 0;
unsigned long firstret_loop_count = 0;
unsigned long secondret_loop_count = 0;

unsigned long ble_connect_errors = 0;  // no BLE connect after 40 s wait time
unsigned long nfc_read_errors = 0;     // e. g. no sensor in range
unsigned long nfc_scan_count = 0;      // how many scans?
unsigned long nfc_inv_cmd_count = 0;   // how much SetInventroy commands?

unsigned long nfc_prot_set_cmd_errors = 0;
unsigned long nfc_inv_cmd_errors = 0;
unsigned long nfc_read_mem_errors = 0;
unsigned long resend_pkts_events = 0;
unsigned long queue_events = 0;

unsigned long resend_wakeup_cnt = 0;

static boolean show_ble = 1;        // what is shown in serial monitor
static boolean show_state = 1;      // show print_state() infos, disabled ftm

boolean sensor_oor;                 // sensor out of range
int ble_answer;                     // char from BLE reeived?
boolean ble_lost_processing = 1;    // send AT+RESET after OK+LOST?

// absolute progam run time in s counting also sleep() phase
unsigned long prg_run_time = 0;     // in sec
unsigned long next_normal_wakeup;
int cons_loop_wo_ble = 0;           // consequtive loops w/o BLE connect

unsigned long loop_start_time;      // loop starting time in ms
unsigned long loop_time;            // current loop duration in ms
unsigned long ble_start_time = 0;   // time when BLE starts ms
unsigned long ble_connect_time = 0; // time up to BLE connection in ms

float avg_sleepTime = 0;            // average sleep time
float bleTime = 0;                  // average arduino+BLE time
float bleNFCTime = 0;

unsigned long var_sleepTime;        // howlong do we want to sleep / s
unsigned long queuesendtime;
unsigned long now;
boolean evtflg1;
boolean evtflg2;
int get_packet_result;
unsigned long next_normal_start;  // next normal 5 min start time

#define NORMAL 0
#define ST_RESEND 1
#define ND_RESEND 2

int loop_state;                   // state machine for loop

boolean v_command = 0;              // v command received?

#ifdef DB_VOLTAGE
#define VOLTAGE_INTERVAL  24
#endif

boolean x_command = 0;              // X command received?

int pkt_retries;

unsigned long no_nfc_reading = 0;
unsigned long bg_is_null = 0;

int spikeCount = 0;
int trendSpikeCount = 0;
int averageSpikeCount = 0;
int glucoseWasChanged = 0;

int simuBGindex = 0;

unsigned long arduino_ontime = 0;
unsigned long arduino_start_time = 0;
unsigned long ble_ontime = 0;
//unsigned long ble_activated = 0;
unsigned long nfc_ontime = 0;
unsigned long nfc_start_time = 0;

unsigned long last_arduino_ontime = 0;
unsigned long last_ble_ontime = 0;
unsigned long last_nfc_ontime = 0;

/* ************************************************************* */
/* *** xBridge2 stuff ****************************************** */
/* ************************************************************* */


// millis() since program start, Arduino millis() are not counting when in sleep mode
unsigned long abs_millis(void)
{
  return (prg_run_time * 1000 + (millis() - loop_start_time));
}

/* **************************************************************** */
/* ********* modified LimiTTer code ******************************* */
/* **************************************************************** */

// initialize the hardware
void setup() {
  // NFC part
  pinMode(IRQPin, OUTPUT);
  digitalWrite(IRQPin, HIGH);
  pinMode(SSPin, OUTPUT);
  digitalWrite(SSPin, HIGH);
  pinMode(NFCPin1, OUTPUT);
  digitalWrite(NFCPin1, HIGH);
  pinMode(NFCPin2, OUTPUT);
  digitalWrite(NFCPin2, HIGH);
  pinMode(NFCPin3, OUTPUT);
  digitalWrite(NFCPin3, HIGH);

 

  // NFC part
  /* not needed here, is done in SPI.begin(), acoording to Bert Roode, 170302
      pinMode(MOSIPin, OUTPUT);
      pinMode(SCKPin, OUTPUT);
  */
  // try this first
  Serial.begin(9600);

  arduino_start_time = millis();
}

/* *********************************************************** */
/* *** NFC LimiTTer code, only small modifications *********** */
/* *********************************************************** */

void SetProtocol_Command() {
  unsigned long ct;

//  print_state(F("do SetProtocol_Command() ..."));

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);

  ct = millis();
  digitalWrite(SSPin, LOW);
  while ( RXBuffer[0] != 8 )
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
      //print_state(F("poll SetProtocol_Command not successfull"));
      Serial.println("poll SetProtocol_Command not successfull");
      break;
    }

  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
  {
//#ifdef DB_PROCESSING
    //print_state(F("Protocol Set Command OK"));
    Serial.println("Protocol Set Command OK");
//#endif
    NFCReady = 1; // NFC is ready
  }
  else
  {
    //print_state(F("Protocol Set Command FAIL"));
    Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
    nfc_prot_set_cmd_errors++;
  }
//  print_state(F("done SetProtocol_Command() ..."));
}

void Inventory_Command() {
  unsigned long ct;
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);

  ct = millis();
  digitalWrite(SSPin, LOW);
  while (RXBuffer[0] != 8)
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    if ( (millis() - ct) > 1000) {
     // print_state(F("poll Inventory_Command not successfull"));
      break;
    }
  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (byte i = 0; i < RXBuffer[1]; i++)
    RXBuffer[i + 2] = SPI.transfer(0); // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)  // is response code good?
  {
//#ifdef DB_PROCESSING
   // print_state(F("Sensor in range ... OK"));
   Serial.println("Sensor in range ... OK");
//#endif
    NFCReady = 2;
    sensor_oor = 0;
  }
  else
  {
    //print_state(F("Sensor out of range"));
    Serial.println("Sensor out of range");
    NFCReady = 1;
    nfc_inv_cmd_errors++;
    sensor_oor = 1;
  }
}

float Read_Memory(boolean *readDone)
{
  byte oneBlock[8];
  String hexPointer = "";
  String trendValues = "";
  String hexMinutes = "";
  String elapsedMinutes = "";
  float trendOneGlucose;
  float trendTwoGlucose;
  float currentGlucose = 0;  // initialise to avoid compiler warning
  float shownGlucose;
  float averageGlucose = 0;
  int glucosePointer;
  int validTrendCounter = 0;
  float validTrend[16];
  unsigned long ct;
  byte readError = 0;
  int readTry;

//  print_state(F("entering Read_Memory()"));

  // see also http://www.marcelklug.de/2015/04/15/freestyle-libre-ein-offenes-buch/
  // 15 min trend values in 8 byte blocks #3 - #15
  for ( int b = 3; b < 16; b++) {
    readTry = 0;
    do {
      readError = 0;
      // read single block
      digitalWrite(SSPin, LOW);
      SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
      SPI.transfer(0x04);  // Send Receive CR95HF command
      SPI.transfer(0x03);  // length of data that follows
      SPI.transfer(0x02);  // request Flags byte
      SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
      SPI.transfer(b);     // memory block address
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // wait for BM019 answer, test for possible timeout
      ct = millis();
      digitalWrite(SSPin, LOW);
      while (RXBuffer[0] != 8) {
        RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
        RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
        if ( (millis() - ct) > 1000) {
          //print_state(F("poll failure in Read_Memory()"));
          Serial.println("poll failure in Read_Memory()");
          break;
        }
      }
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // read the received data block
      digitalWrite(SSPin, LOW);
      SPI.transfer(0x02);             // SPI control byte for read
      RXBuffer[0] = SPI.transfer(0);  // response code
      RXBuffer[1] = SPI.transfer(0);  // length of data
      for (byte i = 0; i < RXBuffer[1]; i++)
        RXBuffer[i + 2] = SPI.transfer(0); // data
      if ( RXBuffer[0] != 128 )
        readError = 1;
      digitalWrite(SSPin, HIGH);
      delay(1);
  
      // copy to convert
      for (int i = 0; i < 8; i++)
        oneBlock[i] = RXBuffer[i + 3];
  
      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for (; pin < oneBlock + 8; pout += 2, pin++) {
        pout[0] = hex[(*pin >> 4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
      }
      pout[0] = 0;
  
      if ( !readError )
      {
        // trendValues contains all data in a row
        trendValues += str;
        // VW - debug
//#ifdef DB_NFC_DATA
       // print_state(F("")); Serial.print(str);
       Serial.println(str);
//#endif
      }
      readTry++;
    } while ( (readError) && (readTry < MAX_NFC_READTRIES) );
  } /* read memory blocks 3 ... 15 */
}
//  print_state(F("trendValues filled"));

  /* *********************************************************************************** */

 

/* ************************************************** */
/* *** NFC handling *** */
/* ************************************************** */

// new function to fit Limitter in xBridge concept, called from loop(), formerly in setup()
void configNFC(void)
{
  SPI.begin();

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode
  delay(10);
  digitalWrite(IRQPin, LOW);

  SPI.end();    // according to Bert Roode 170227. Nested SPI.begin() and SPI.end()
}

void wakeNFCUp(void)
{
  // print_state(F("mem avail "));
  // Serial.print(freeMemory());

  digitalWrite(NFCPin1, HIGH);
  digitalWrite(NFCPin2, HIGH);
  digitalWrite(NFCPin3, HIGH);
  digitalWrite(IRQPin, HIGH);

  SPI.begin();

  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV32);
  delay(10);
  digitalWrite(IRQPin, LOW);
  delayMicroseconds(100);
  digitalWrite(IRQPin, HIGH);
  delay(10);
  digitalWrite(IRQPin, LOW);

  NFCReady = 0;

  nfc_start_time = millis();
}

void shutNFCDown(void)
{
  // print_state(F("mem avail "));
  // Serial.print(freeMemory());

  SPI.end();

  digitalWrite(MOSIPin, LOW);
  digitalWrite(SCKPin, LOW);
  digitalWrite(NFCPin1, LOW); // Turn off all power sources completely
  digitalWrite(NFCPin2, LOW); // for maximum power save on BM019.
  digitalWrite(NFCPin3, LOW);
  digitalWrite(IRQPin, LOW);

  nfc_ontime += (millis() - nfc_start_time);
}



// main processing loop
void loop(void)
{
  int i, j;
 
  loop_start_time = millis();             // log the current time
 // init_command_buff(&command_buff);       //initialise the command buffer


  

  configNFC();                // initialize and configure the NFC module

 
  

    // *********** NFC ***************************************

    // get BG reading, not in case of last tranfer failed
    int zz;
   
          wakeNFCUp();
         
      // was sensor out of range?
      if ( sensor_oor )
        Serial.println("Sensor out of range");
    
    // ******************* handling **************************
    if (zz=1) {
 SetProtocol_Command();
 Inventory_Command();
 Read_Memory('1');
 
    }
    zz=zz+1;
    
   
}



/*
NFC Communication with the Solutions Cubed, LLC BM019 
and an Arduino Uno.  The BM019 is a module that
carries ST Micro's CR95HF, a serial to NFC converter.

Wiring:
 Arduino          BM019
 IRQ: Pin 9       DIN: pin 2
 SS: pin 10       SS: pin 3
 MOSI: pin 11     MOSI: pin 5 
 MISO: pin 12     MISO: pin4
 SCK: pin 13      SCK: pin 6
 
 */

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
//#include <SoftwareSerial.h>


const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse
byte TXBuffer[40];    // transmit buffer
byte RXBuffer[40];    // receive buffer
byte NFCReady = 0;  // used to track NFC state
int sensorMinutesElapse; // per memorizzare i minuti trascorsi dall'attivazione del sensore 
int minutiTrascorsi;  //variabile per memorizzare ultimo momento di scansione glicemia
int LastGlucoseValue; // varabile usata per determinare la velocità di variazione della glicemia
int diff;
String velocita; // ci metto le frecce di variazione

void setup() {
    pinMode(IRQPin, OUTPUT);
    digitalWrite(IRQPin, HIGH); // Wake up pulse
    pinMode(SSPin, OUTPUT);
    digitalWrite(SSPin, HIGH);

    Serial.begin(9600);
    SPI.begin();
    SPI.setDataMode(SPI_MODE0);
    SPI.setBitOrder(MSBFIRST);
    SPI.setClockDivider(SPI_CLOCK_DIV32);
 
 // The CR95HF requires a wakeup pulse on its IRQ_IN pin
 // before it will select UART or SPI mode.  The IRQ_IN pin
 // is also the UART RX pin for DIN on the BM019 board.
 
    delay(20);                      // send a wake up
    digitalWrite(IRQPin, LOW);      // pulse to put the 
    delayMicroseconds(100);         // BM019 into SPI
    digitalWrite(IRQPin, HIGH);     // mode 
    delay(20);
    digitalWrite(IRQPin, LOW);
}


void Poll()
{
  // step 2, poll for data ready
  digitalWrite(SSPin, LOW);
  while(RXBuffer[0] != 8)
    {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
  digitalWrite(SSPin, HIGH);
  delay(1); //era a 1
}

/* IDN_Command identifies the CR95HF connected to the Arduino.
This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display the CR95HF ID number and CRC code.  This rountine is 
not that useful in using the NFC functions, but is a good way to 
verify connections to the CR95HF. 
*/
void IDN_Command()
 {
 byte i = 0;
// step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0);  // SPI control byte to send command to CR95HF
  SPI.transfer(1);  // IDN command
  SPI.transfer(0);  // length of data that follows is 0
  digitalWrite(SSPin, HIGH);
  delay(1); // era a 1
 
// step 2, poll for data ready
// data is ready when a read byte
// has bit 3 set (ex:  B'0000 1000')

 Poll();


// step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data (Device ID in Ascii), Rom CRC
  digitalWrite(SSPin, HIGH);
  delay(1); //era a 1

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 15))
  {  
    Serial.println("IDN COMMAND-");  //
    Serial.print("RESPONSE CODE: ");
    Serial.print(RXBuffer[0]);
    Serial.print(" LENGTH: ");
    Serial.println(RXBuffer[1]);
    Serial.print("DEVICE ID: ");
    for(i=2;i<(RXBuffer[1]);i++)
    {
      Serial.print(RXBuffer[i],HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
    Serial.print("ROM CRC: ");
    Serial.print(RXBuffer[RXBuffer[1]],HEX);
    Serial.print(RXBuffer[RXBuffer[1]+1],HEX);
    Serial.println(" ");
  }
  else
    Serial.println("BAD RESPONSE TO IDN COMMAND!");

  Serial.println(" ");
}

/* SetProtocol_Command programs the CR95HF for
ISO/IEC 15693 operation.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display successful programming. 
*/
void SetProtocol_Command()
 {
 byte i = 0;
 
// step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC : 10110000 = 0x0D
  digitalWrite(SSPin, HIGH);
  delay(10);    // aumentato delay a 10 non fa nulla
 
// step 2, poll for data ready

  Poll();

// step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read        
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))
  {
     Serial.println("PROTOCOL SET-");  //
     NFCReady = 1; // NFC is ready
  }
  else
  {
     Serial.println("BAD RESPONSE TO SET PROTOCOL");
     NFCReady = 0; // NFC not ready
  }
  Serial.println(" ");
}

/* Inventory_Command chekcs to see if an RF
tag is in range of the BM019.

This requires three steps.
1. send command
2. poll to see if CR95HF has data
3. read the response

If the correct response is received the serial monitor is used
to display the the RF tag's universal ID.  
*/
void Inventory_Command()
 {
 byte i = 0;

// step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1); // era  a 1
 
// step 2, poll for data ready
// data is ready when a read byte
// has bit 3 set (ex:  B'0000 1000')

 Poll();

// step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read         
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (i=0;i<RXBuffer[1];i++)      
      RXBuffer[i+2]=SPI.transfer(0);  // data
  digitalWrite(SSPin, HIGH);
  delay(1); //era a 50

  if (RXBuffer[0] == 128)
  {  
    Serial.println("TAG DETECTED - Freestyle libre trovato!");
    Serial.print("UID: ");
    for(i=11;i>=4;i--)
    {
      Serial.print(RXBuffer[i],HEX);
      Serial.print(" ");
    }
    Serial.println(" ");
    //NFCReady = 2;
  }
  else
    {
    Serial.print("NO TAG IN RANGE - ");
    Serial.print("RESPONSE CODE: ");
    Serial.println(RXBuffer[0],HEX);
    //NFCReady=1;
    }
  Serial.println(" ");
}

// Funzione per il comando di richiesta informazione del TAG

void InfoTag_Command()
  {
    byte i = 0;
   // byte ii = 0;
    //Step 1 mando il comando
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00); //SPI control byte da inviare al CR95HF 
    SPI.transfer(0x04); // Information command
    SPI.transfer(0x02); // Lunghezza dati
    SPI.transfer(0x02); // Data Request Flags
    SPI.transfer(0x2B); // Data block command
    digitalWrite(SSPin, HIGH);
    delay(1);  // era a 1

    //Step 2 Aspetto finchè i dati sono pronti (in questo caso è settato dal CR95HF il terzo bit

    Poll();

    // Step 3, Leggo i dati
    digitalWrite (SSPin, LOW);
    SPI.transfer(0x02); //SPI control byte for read
    RXBuffer[0] = SPI.transfer (0);  // response code (0x80)
    RXBuffer[1] = SPI.transfer (0);  // lunghezza dei dati (0x12) o (0x13)
    for (i=0; i<RXBuffer[1];i++)
      RXBuffer[i+2]=SPI.transfer(0); // memorizzo i dati in RXBuffer
    digitalWrite (SSPin, HIGH);
    delay(1);//era a 1

    if (RXBuffer[0] == 128 )
    {
      Serial.println("Sensore rilevato!");
      Serial.print("Lunghezza dati : ");
      Serial.println(RXBuffer[1],DEC);
      Serial.print("UID: ");
      for(i=11;i>=4;i--)
        {
          Serial.print(RXBuffer[i],HEX);
          Serial.print(" ");
        }
      Serial.println(" ");
      Serial.print(" Dimensione Memoria : ");
      Serial.print(RXBuffer[14],DEC);
      Serial.print(" ");
      Serial.println(RXBuffer[15],DEC);
     // Serial.print(" IC Ref : ");
     // ii = RXBuffer[1]-3;
     // Serial.println(RXBuffer[ii],DEC);
      
      
    }
    else
   {
      Serial.print("SENSORE NON IN RANGE - ");
      Serial.print("RESPONSE CODE: ");
      Serial.println(RXBuffer[0],HEX);
   }
  Serial.println(" ");
}

void leggiMemoria()
{
  ///////////////////////////////////////////////////////////////dichiarazioni///////////////////
  byte readError = 0;
  byte i =0;
  byte oneBlock[8]; //in questo vettore memorizzo una riga da 8 valori
  
  String trendValues = ""; //Conterrà tutti i 208 valori di memoria del sensore (13 righe x (2x8))
  String elapsedMinutes = "";
  String hexPointer = "";
  String hexMinutes = "";
  
  float trendOneGlucose;
  float trendTwoGlucose;
  float currentGlucose;
  float shownGlucose;
  float averageGlucose;
  float validTrend[16];
  float currentTemp;
  float NTC;
  float Temp;
  
  int glucosePointer;
  int posGlucNow;
  int validTrendCounter;
  int a =0 ;
  ///////////////////////////////////////////////////////////////////////////////////////////////
  
  // step 1 mando i comandi
  for ( int b = 3; b < 16; b++) { //leggo i blocchi di memoria da 3 a 16  
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    SPI.transfer(0x04);  // Send Receive CR95HF command
    SPI.transfer(0x03);  // length of data that follows
    SPI.transfer(0x02);  // request Flags byte
    SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
    SPI.transfer(b);  // memory block address
    digitalWrite(SSPin, HIGH);
    delay(1);//era a 20
  
  // step 2 aspetto finchè i dati sono pronti
  Poll();

  // step 3 leggo i dati
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02); //SPI byte di controllo per lettura
  RXBuffer[0]=SPI.transfer(0); //response o result code
  RXBuffer[1]=SPI.transfer(0); //lunghezza dei dati
  for (i = 0; i<RXBuffer[1];i++)
        RXBuffer[i+2]=SPI.transfer(0); //dati    
  digitalWrite (SSPin, HIGH);
  delay(1);//era a 20 
  
  if (RXBuffer[0] == 128) //La risposta è buona?
  {
      //NFCReady=2;  
      readError = 0;// era commentato
      for (a = 0;a<8;a++){
        //delay(50); //da levare
        Serial.print(RXBuffer[a+3],DEC);  //DA COMMENTARE
        //Serial.print(RXBuffer[a+3],HEX);
        oneBlock[a]=RXBuffer[a+3];
        Serial.print(" "); //Da commentare
      }
      //inizio la decodifica strana 
      char str[24];
      unsigned char * pin = oneBlock; //pin punta allo stesso indirizzo del vettore oneBlock
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for (; pin < oneBlock+8; pout+=2, pin++) {
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[*pin & 0xF];
      }
      pout[0]=0;
      //Serial.print("Valore della riga memoria---->");
      //Serial.println(str);
      trendValues +=str; //contiene tutti i dati della memoria (compattati) ATTENZIONE è importante!!!
  }
  else {    
    Serial.println("Lettura memoria fallita!");   
    readError=1;     
  }
  Serial.println(" "); //da commentare
  }
 //lettura memoria blocco 39
 if (!readError){
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows
  SPI.transfer(0x02);  // request Flags byte
  SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
  SPI.transfer(39);  // memory block address 39 : informazioni sul tempo trascorso
  digitalWrite(SSPin, HIGH);
  delay(1);//era a 20

  // step 2 aspetto finchè i dati sono pronti
  Poll();

  // step 3 leggo i dati
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02); //SPI byte di controllo per lettura
  RXBuffer[0]=SPI.transfer(0); //response o result code
  RXBuffer[1]=SPI.transfer(0); //lunghezza dei dati
  for (i = 0; i<RXBuffer[1];i++)
        RXBuffer[i+2]=SPI.transfer(0); //dati    
  digitalWrite (SSPin, HIGH);
  delay(1);  //ERA 20
  
  if (RXBuffer[0] == 128) //La risposta è buona?
  { 
      //readError=0;
      //Serial.println ("LEGGO IL BLOCCO DI MEMORIA 39");
      for (a = 0;a<8;a++){
        //Serial.print(RXBuffer[a+3],HEX);
        oneBlock[a]=RXBuffer[a+3];
        //Serial.print(" ");
      }
      //metto in str l'intera riga di memoria 
      char str[24];
      unsigned char * pin = oneBlock; //pin punta allo stesso indirizzo del vettore oneBlock
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for (; pin < oneBlock+8; pout+=2, pin++) {
        pout[0] = hex[(*pin>>4) & 0xF];
        pout[1] = hex[*pin & 0xF];
      }
      pout[0]=0;
      elapsedMinutes += str; // metto la riga letta del blocco memoria 39 nella stringa elapsedMinutes
  }
  else {    
    Serial.println("Lettura memoria fallita!");  
    readError=1;      // errore nella lettura
  }
  Serial.println(" ");
 }
  if (!readError)
  {
    hexMinutes=elapsedMinutes.substring(10,12)+elapsedMinutes.substring(8,10); //prendo il tempo trascorso in minuti dal momento dell'attivazione del sensore
    //per sapere i giorni basta dividere il valore (trasformato in decimale) e dividerlo per 1440
    Serial.print("valore di hexMinutes  ----> ");
    Serial.println(hexMinutes);
    hexPointer = trendValues.substring(4,6);
    //Serial.print("valore di hexPointer  ----> ");
    //Serial.println(hexPointer);
    sensorMinutesElapse = strtoul (hexMinutes.c_str(), NULL, 16); // la funzione c_str() converte la string in un array di char per poter poi essere convertita in numero con strtoul
    
    Serial.println("Minuti trascorsi da attivazione sensore: "+String( sensorMinutesElapse));
    glucosePointer = strtoul(hexPointer.c_str(), NULL, 16); // trasforma in intero il valore di hexPointer nella notazione decimale

    Serial.println("");
    Serial.print("GlucosePointer:.......... ");
    Serial.print(glucosePointer);
    Serial.println("");

    switch (glucosePointer) {
      case 0 :
        {
          posGlucNow=188;
        }
        break;
      case 1:
         {
          posGlucNow=8; 
        } 
        break;
      case 2:
        {
          posGlucNow=20;
        } 
        break;
      case 3:
        {
          posGlucNow=32;
        } 
        break;
      case 4:
        {
          posGlucNow=44;
        } 
        break;
      case 5:
        {
          posGlucNow=56;
        } 
        break;
      case 6:
        {
          posGlucNow=68;
        } 
        break;
      case 7:
        {
          posGlucNow=80;
        } 
        break;
      case 8:
        {
          posGlucNow=92;
        } 
        break;
      case 9:
        {
          posGlucNow=104;
        } 
        break;
      case 10:
        {
         posGlucNow=116;
        } 
        break;
      case 11:
        {
          posGlucNow=128;
        } 
        break;
      case 12:
        {
          posGlucNow=140;
        } 
        break;
       case 13:
        {
          posGlucNow=152;
        } 
        break;
       case 14:
        {
          posGlucNow=164;
        } 
        break;
       case 15:
        {
          posGlucNow=176;
        } 
        break;
       default:
        Serial.println("Errore glucosePointer");
         // if nothing else matches, do the default
         // default is optional
      break;
    }
    if ((sensorMinutesElapse-minutiTrascorsi)>0 && (!readError)){
      Serial.println("/////////////////////////////////////////////////////////////////////////////");
      String trendNow = trendValues.substring(posGlucNow+2,posGlucNow+4) + trendValues.substring(posGlucNow,posGlucNow+2);//<--------Estrae glicemia
      Serial.print("valore variabile trendNow:");
      Serial.println(trendNow);
      currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL ,16));// trasforma in intero decimale il valore di trendNow 
      //Serial.print("valore glucosio:....... ");
      //Serial.print(currentGlucose);
      //Serial.println(" mg/dL");
      ///////////Calcolo il valore della temperatura
      String trendNowTemp = trendValues.substring(posGlucNow+8,posGlucNow+10) + trendValues.substring(posGlucNow+6,posGlucNow+8);//<---------------Estrae Temperatura
      //Serial.print("valore variabile trendNowTemp:");
      //Serial.println(trendNowTemp);
      //Serial.print("valore di strtoul(trendNowTemp.c_str(),NULL,16)...............:");
      //Serial.println(strtoul(trendNowTemp.c_str(), NULL, 16));
      currentTemp = 0x3FFF & strtoul(trendNowTemp.c_str(), NULL ,16);// trasforma in intero decimale il valore di trendNow (maschero con 3FFF perchè l'ADC è a 14 bit)
      NTC = currentTemp * 11.44 ; // ho il valore ohmico della NTC
      Temp = 22.124 * log(309443.81/NTC);
      Serial.println("/////////////////////////////////////////////////////////////////////////////");
      Serial.println("Minuti trascorsi da ultima scansione:"+String(sensorMinutesElapse - minutiTrascorsi));
      if ((sensorMinutesElapse -  minutiTrascorsi) == 1){
        diff=currentGlucose-LastGlucoseValue;   
        if (diff < -2) {
          velocita="vb";
        }
        else if (diff > 2){
          velocita="va";
        }
        else if  (diff >1 and diff <2 ){
          velocita="oa";
        }
        else if  (diff <-1 and diff >-2 ){
          velocita="ob";
        }
        else if  (abs(diff) <=1){
          velocita="Glice stabile!!";
        }
      }
      else velocita="---";
      Serial.print("Valore glicemia:....... ");
      Serial.print(currentGlucose);
      Serial.print(" mg/dL  ");
      Serial.println (velocita);
      Serial.print("Valore della Temperatura:....... ");
      Serial.println(Temp);
      LastGlucoseValue=currentGlucose;
      minutiTrascorsi=sensorMinutesElapse; // mette i minuti all'ultima scansione fatta
      /////////////////////////////////////////////////////////////////////////////////
      //INVIA DATO AL BLE//////////////////////////////////////////////////////////////
      /////////////////////////////////////////////////////////////////////////////////
      //Serial.begin(115200);  //initial the Serial
      //Serial.write(inserire il valore della glice) 
      //Serial.println();   //print line feed character   
      delay(58000);
    }
  } 
}

float Glucose_Reading(unsigned int val) {
        int bitmask = 0x0FFF;
        //return ((val & bitmask) / 10.4);
        return ((val & bitmask) / 8.5);
        //return (((val & bitmask)-28.96) / 9.391);
}
 
void loop() 
{ 
  switch (NFCReady){ 
  case 0: IDN_Command();  // reads the CR95HF ID
          delay(1000);
          SetProtocol_Command(); // ISO 15693 settings
          delay(1000); 
          Inventory_Command();
          delay(1000);
          break;
  case 1: InfoTag_Command();
          delay(1000);
          leggiMemoria();
          delay(1000);
          break;
   }
}


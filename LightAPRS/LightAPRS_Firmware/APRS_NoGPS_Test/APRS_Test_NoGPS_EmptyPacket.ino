#include <LibAPRS.h>        //Modified version of https://github.com/markqvist/LibAPRS
#include <SoftwareSerial.h>
#include <LowPower.h>       //https://github.com/rocketscream/Low-Power
#include <Wire.h>
#include <Adafruit_BMP085.h>//https://github.com/adafruit/Adafruit-BMP085-Library
#include <avr/wdt.h>

// Pin Definitions
#define RfPDPin     19
#define RfPwrHLPin  21
#define RfPttPin    20
#define BattPin     A2
#define PIN_DRA_RX  22
#define PIN_DRA_TX  23
#define TX_LED      7

// Constants
#define ADC_REFERENCE REF_3V3
#define OPEN_SQUELCH false
#define DraHighVolt 8.0    // min Volts for radio module (DRA818V) to transmit (TX) 1 Watt

// Pin Control Macros
#define RfON          digitalWrite(RfPDPin, HIGH)
#define RfOFF         digitalWrite(RfPDPin, LOW)
#define RfPwrHigh     pinMode(RfPwrHLPin, INPUT)
#define RfPwrLow      pinMode(RfPwrHLPin, OUTPUT);digitalWrite(RfPwrHLPin, LOW)
#define RfPttON       digitalWrite(RfPttPin, HIGH)//NPN
#define RfPttOFF      digitalWrite(RfPttPin, LOW)
#define AprsPinInput  pinMode(12,INPUT);pinMode(13,INPUT);pinMode(14,INPUT);pinMode(15,INPUT)
#define AprsPinOutput pinMode(12,OUTPUT);pinMode(13,OUTPUT);pinMode(14,OUTPUT);pinMode(15,OUTPUT)

#define DEVMODE // Development mode. Uncomment to enable for debugging.

// APRS Configuration
char  CallSign[7]="9V1AE"; //DO NOT FORGET TO CHANGE YOUR CALLSIGN
int   CallNumber=11; //SSID http://www.aprs.org/aprs11/SSIDs.txt
char  Symbol='O'; // '/O' for balloon, '/>' for car, for more info : http://www.aprs.org/symbols/symbols-new.txt
bool alternateSymbolTable = false ; //false = '/' , true = '\'
char Frequency[9]="144.3900"; //default frequency. 144.3900 for US, 144.8000 for Europe
char comment[50] = "http://www.lightaprs.com"; // Max 50 char
char StatusMessage[50] = "LightAPRS by TA9OHC & TA2MUN"; 

// Timing and Power Settings
unsigned int BeaconWait=20;  //seconds sleep for next beacon (TX).
unsigned int BattWait=60;    //seconds sleep if super capacitors/batteries are below BattMin
float BattMin=4.5;        // min Volts to wake up.
boolean aliveStatus = true; //for tx status message on first wake-up just once.

// APRS Path Settings
byte  Wide1=1; // 1 for WIDE1-1 path
byte  Wide2=1; // 1 for WIDE2-1 path
int pathSize=2; // 2 for WIDE1-N,WIDE2-N ; 1 for WIDE2-N

// Global Variables
static char telemetry_buff[100];// telemetry buffer
uint16_t TxCount = 1;
Adafruit_BMP085 bmp;
String serialCommand;

// Function Declarations
float readBatt();
void sendStatus();
void updateTelemetry();
void freeMem();
byte configDra818(char *freq);
void sleepSeconds(int sec);

void setup() {
  wdt_enable(WDTO_8S);
  analogReference(INTERNAL2V56);
  pinMode(RfPDPin, OUTPUT);
  pinMode(RfPwrHLPin, OUTPUT);
  pinMode(RfPttPin, OUTPUT);
  pinMode(BattPin, INPUT);
  pinMode(PIN_DRA_TX,INPUT);
  pinMode(TX_LED, OUTPUT);
  digitalWrite(TX_LED, LOW);
  RfOFF;
  RfPwrLow;
  RfPttOFF;
  AprsPinInput;
  Serial.begin(57600);
#if defined(DEVMODE)
  Serial.println(F("Start"));
#endif
  APRS_init(ADC_REFERENCE, OPEN_SQUELCH);
  APRS_setCallsign(CallSign,CallNumber);
  APRS_setDestination("APLIGA", 0);
  APRS_setMessageDestination("APLIGA", 0);
  APRS_setPath1("WIDE1", Wide1);
  APRS_setPath2("WIDE2", Wide2);
  APRS_useAlternateSymbolTable(alternateSymbolTable); 
  APRS_setSymbol(Symbol);
  APRS_setPreamble(350UL);  
  APRS_setPathSize(pathSize);
  configDra818(Frequency);
  bmp.begin();
}

void loop() {
  wdt_reset();
  if (readBatt() > BattMin) {
    if(aliveStatus){
      #if defined(DEVMODE)
        Serial.println(F("Sending"));
      #endif
      sendStatus();
      #if defined(DEVMODE)
        Serial.println(F("Sent"));
      #endif
      aliveStatus = false;
    }
    updateTelemetry();
    sendStatus();
    freeMem();
    Serial.flush();
    sleepSeconds(BeaconWait);
  } else {
    sleepSeconds(BattWait);
  }
}

void aprs_msg_callback(struct AX25Msg *msg) {
  //do not remove this function, necessary for LibAPRS
}

void sleepSeconds(int sec) {  
  RfOFF;
  RfPttOFF;
  Serial.flush();
  wdt_disable();
  for (int i = 0; i < sec; i++) {
    LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_ON);   
  }
  wdt_enable(WDTO_8S);
}

byte configDra818(char *freq)
{
  SoftwareSerial Serial_dra(PIN_DRA_RX, PIN_DRA_TX);
  Serial_dra.begin(9600);
  RfON;
  char ack[3];
  int n;
  delay(2000);
  char cmd[50];
  sprintf(cmd, "AT+DMOSETGROUP=0,%s,%s,0000,4,0000", freq, freq);
  Serial_dra.println(cmd);
  ack[2] = 0;
  while (ack[2] != 0xa)
  {
    if (Serial_dra.available() > 0) {
      ack[0] = ack[1];
      ack[1] = ack[2];
      ack[2] = Serial_dra.read();
    }
  }
  Serial_dra.end();
  RfOFF;
  pinMode(PIN_DRA_TX,INPUT);
#if defined(DEVMODE)
  if (ack[0] == 0x30) Serial.println(F("Frequency updated...")); else Serial.println(F("Frequency update error!"));
#endif
  return (ack[0] == 0x30) ? 1 : 0;
}

void updateTelemetry() {
  telemetry_buff[3] += '/';
  telemetry_buff[7] = '/';
  telemetry_buff[8] = 'A';
  telemetry_buff[9] = '=';
  telemetry_buff[16] = ' ';
  sprintf(telemetry_buff + 17, "%03d", TxCount);
  telemetry_buff[20] = 'T';
  telemetry_buff[21] = 'x';
  telemetry_buff[22] = 'C';
  telemetry_buff[23] = ' '; float tempC = bmp.readTemperature();
  dtostrf(tempC, 6, 2, telemetry_buff + 24);
  telemetry_buff[30] = 'C';
  telemetry_buff[31] = ' '; float pressure = bmp.readPressure() / 100.0; //Pa to hPa
  dtostrf(pressure, 7, 2, telemetry_buff + 32);
  telemetry_buff[39] = 'h';
  telemetry_buff[40] = 'P';
  telemetry_buff[41] = 'a';
  telemetry_buff[42] = ' ';
  dtostrf(readBatt(), 5, 2, telemetry_buff + 43);
  telemetry_buff[48] = 'V';
  telemetry_buff[49] = ' ';
  telemetry_buff[52] = 'S';
  telemetry_buff[53] = ' ';
  sprintf(telemetry_buff + 54, "%s", comment);
#if defined(DEVMODE)
  Serial.println(telemetry_buff);
#endif
}

void sendStatus() {
  RfPwrLow; //DRA Power 0.5 Watt
  RfON;
  delay(2000);  // Wait for radio to stabilize
  
  // Prepare the APRS packet
  char statusBuffer[100];
  float tempC = bmp.readTemperature();
  float pressure = bmp.readPressure() / 100.0;
  float voltage = readBatt();
  
  // Format the status message with fixed positions for better readability
  sprintf(statusBuffer, "!%.6f/%.6f_%03d/%03d/A=%.1f T=%.1fC P=%.1fhPa V=%.1fV %s",
          0.0, 0.0,  // No GPS coordinates
          0, 0,      // No heading/speed
          tempC,     // Temperature
          tempC,     // Temperature again for redundancy
          pressure,  // Pressure
          voltage,   // Voltage
          comment);
  
  // Turn on PTT and wait for radio to stabilize
  RfPttON;
  delay(100);
  
  // Send the APRS packet
  APRS_sendStatus(statusBuffer, strlen(statusBuffer));
  
  // Wait for transmission to complete
  delay(500);
  
  // Turn off PTT and radio
  RfPttOFF;
  delay(100);
  RfOFF;
  
#if defined(DEVMODE)
  Serial.println(F("Status sent"));
  Serial.println(statusBuffer);
#endif
  TxCount++;
}

float readBatt() {
  float R1 = 560000.0; // 560K
  float R2 = 100000.0; // 100K
  float value = 0.0;
  do { 
    value =analogRead(BattPin);
    delay(5);
    value =analogRead(BattPin);
    value=value-8;
    value = (value * 2.56) / 1024.0;
    value = value / (R2/(R1+R2));
  } while (value > 16.0);
  return value ;
}

void freeMem() {
#if defined(DEVMODE)
  Serial.print(F("Free RAM: ")); Serial.print(freeMemory()); Serial.println(F(" byte"));
#endif
}

static void printFloat(float val, bool valid, int len, int prec)
{
#if defined(DEVMODE)
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i = flen; i < len; ++i)
      Serial.print(' ');
  }
#endif
}

static void printInt(unsigned long val, bool valid, int len)
{
#if defined(DEVMODE)
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i = strlen(sz); i < len; ++i)
    sz[i] = ' ';
  if (len > 0)
    sz[len - 1] = ' ';
  Serial.print(sz);
#endif
}

static void printStr(const char *str, int len)
{
#if defined(DEVMODE)
  int slen = strlen(str);
  for (int i = 0; i < len; ++i)
    Serial.print(i < slen ? str[i] : ' ');
#endif
}
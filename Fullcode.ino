#define BLYNK_TEMPLATE_ID "TMPL3Ud05NJQj"
#define BLYNK_TEMPLATE_NAME "Smart waste Collection and Management System"
#define BLYNK_AUTH_TOKEN "C_YrSf7ewezmf_EZnsLM9NKmkcg8AgFZ"
//THIS EXAMPLE SHOWS HOW VVM501 ESP32 4G LTE MODULE CAN CONNECT TO BLYNK SERVER
//THE DEVICE USES DHT22 TO PUBLISH THE TEMPERATURE AND HUMIDITY VALUES TO BLYNK SERVER
// CONNECTION DIAGRAM BETWEEN DHT22 AND ESP32 4G MODULE
// DHT22    <------------->     ESP32 4G LTE MODULE
// + PIN    <------------->        3V3
// OUT PIN  <------------->        D15
// - PIN    <------------->        GND
//FOR VVM501 PRODUCT DETAILS VISIT www.vv-mobility.com

//#define BLYNK_TEMPLATE_ID "XXXXXXXX" // PASTE YOUR TEMPLATE ID
//#define BLYNK_DEVICE_NAME "XXXXXXXX"  // PASTE YOUR BLYNK DEVICE NAME
//#define BLYNK_AUTH_TOKEN "XXXXXXXXXXXXXXXXXXXXXXXXXXXXX"   // PASTE YOUR BLYNK AUTHENTICATION TOKE

#define RXD2 27     //VVM501 MODULE RXD INTERNALLY CONNECTED
#define TXD2 26     //VVM501 MODULE TXD INTERNALLY CONNECTED
#define powerPin 4  ////VVM501 MODULE ESP32 PIN D4 CONNECTED TO POWER PIN OF A7670C CHIPSET, INTERNALLY CONNECTED

// Select your modem:
#define TINY_GSM_MODEM_SIM7600  //SIMA7670 Compatible with SIM7600 AT instructions

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
//#define BLYNK_HEARTBEAT 30

#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <Arduino.h>
#include <Wire.h>
int rx = -1;
#define SerialAT Serial1
String rxString;
int _timeout;
String _buffer;

BlynkTimer timer;

// You should get Auth Token in the Blynk App.
// Go to the Project Settings (nut icon).
char auth[] = BLYNK_AUTH_TOKEN;

// Your GPRS credentials
// Leave empty, if missing user or pass
char apn[] = "";
char user[] = "";
char pass[] = "";

bool reply = false;
TinyGsm modem(SerialAT);
/* * Ultrasonic Sensor HC-SR04 interfacing with Arduino.
*/
// defining the pins
#define trigPin 15
#define echoPin 2
long duration;
int distance;

const int gasPin = 34;  // Define the pin for the gas sensor

String number = "+917398609492";  //REPLACE WITH YOUR NUMBER
//String number = "+918808260112";  //REPLACE WITH YOUR NUMBER

//**********GPS code
#include <TinyGPS++.h>
#include <HardwareSerial.h>

// Define the RX and TX pins for the GPS module
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

// Create a HardwareSerial object to communicate with the GPS module
HardwareSerial gpsSerial(2);  // Using UART 1 on ESP32

// Create a TinyGPS++ object
TinyGPSPlus gps;
//*************
const int sendInterval = 2000;

void SendMessage() {
  Serial.println("Sending Message");
  SerialAT.println("AT+CMGF=1");  //Sets the GSM Module in Text Mode
  delay(1000);
  //Serial.println ("Set SMS Number");
  SerialAT.println("AT+CMGS=\"" + number + "\"\r");  //Mobile phone number to send message
  delay(1000);
  String SMS = "DUSTBIN FULL TIME TO PICK IT";
  SerialAT.println(SMS);
  delay(100);
  SerialAT.println((char)26);  // ASCII code of CTRL+Z
  delay(1000);
  _buffer = _readSerial();
}
void RecieveMessage() {
  Serial.println("VVM501 AT7670C Read an SMS");
  delay(1000);
  SerialAT.println("AT+CNMI=2,2,0,0,0");  // AT Command to receive a live SMS
  delay(1000);
  Serial.write("Unread Message done");
}
String _readSerial() {
  _timeout = 0;
  while (!SerialAT.available() && _timeout < 12000) {
    delay(13);
    _timeout++;
  }
  if (SerialAT.available()) {
    return SerialAT.readString();
  }
}
void callNumber() {
  SerialAT.print(F("ATD"));
  SerialAT.print(number);
  SerialAT.print(F(";\r\n"));
  _buffer = _readSerial();
  Serial.println(_buffer);
}

void sendSensor() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Prints the distance on the Serial Monitor
  int dustbinH = map(distance, 5, 40, 0, 100);
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("  % ");
  Serial.println(dustbinH);

  Blynk.virtualWrite(V3, (dustbinH));  // WRITING HUMIDITY VALUE TO VIRTUAL PIN V1 IN BLYNK
  if (distance <= 10) {
    SendMessage();
    Serial.println("DUSTBIN FULL TIME TO PICK IT");
    Blynk.virtualWrite(V4, "DUSTBIN FULL TIME TO PICK IT");  // WRITING  VALUE TO VIRTUAL PIN V1 IN BLYNK
  }
  //.............

  int gasValue = analogRead(gasPin);  // Read the gas sensor value
  Serial.print("Gas value: ");
  Serial.println(gasValue);          // Print the gas sensor value
  Blynk.virtualWrite(V0, gasValue);  // WRITING TEMPERATURE VALUE TO VIRTUAL PIN V0 IN BLYNK
  //*****************

  if (SerialAT.available() > 0)
    Serial.write(SerialAT.read());
  delay(100);
  ///*****************
  boolean newData = false;
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (gpsSerial.available()) {
      if (gps.encode(gpsSerial.read())) { newData = true; }
    }
  }

  //If newData is true
  if (newData == true) {
    newData = false;
    Serial.println(gps.satellites.value());
    if (gps.location.isValid() == 1) {
      //String gps_speed = String(gps.speed.kmph());
      Serial.println(gps.location.lat(), 6);
      Serial.println(gps.location.lng(), 6);
      Serial.println(gps.speed.kmph());
      Serial.println(gps.satellites.value());
      Serial.println(gps.altitude.meters(), 0);
      Blynk.virtualWrite(V1, gps.location.lat());  // WRITING TEMPERATURE VALUE TO VIRTUAL PIN V0 IN BLYNK
      Blynk.virtualWrite(V2, gps.location.lng());  // WRITING HUMIDITY VALUE TO VIRTUAL PIN V1 IN BLYNK

    } else {
      Serial.println("No any valid GPS data.");
    }
  } else {
    Serial.println("No new data is received.");
  }

  //delay(sendInterval);
  //
}



void setup() {
  Serial.begin(115200);
  Serial.print("Welcome to my project ");
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  pinMode(powerPin, OUTPUT);
  digitalWrite(powerPin, LOW);

  delay(100);
  SerialAT.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(10000);

  Serial.println("Modem Reset, Please Wait");
  SerialAT.println("AT+CRESET");
  delay(1000);
  SerialAT.println("AT+CRESET");
  delay(20000);  // WAITING FOR SOME TIME TO CONFIGURE MODEM
  SerialAT.flush();

  Serial.println("Echo Off");
  SerialAT.println("ATE0");  //120s
  delay(1000);
  SerialAT.println("ATE0");  //120s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("OK");
  if (rx != -1)
    Serial.println("Modem Ready");
  delay(1000);

  Serial.println("SIM card check");
  SerialAT.println("AT+CPIN?");  //9s
  rxString = SerialAT.readString();
  Serial.print("Got: ");
  Serial.println(rxString);
  rx = rxString.indexOf("+CPIN: READY");
  if (rx != -1)
    Serial.println("SIM Card Ready");
  delay(1000);

  String name = modem.getModemName();
  delay(500);
  Serial.println("Modem Name: " + name);
  Serial.println("Type s to send an SMS, r to receive an SMS, and c to make a call");

  Blynk.begin(auth, modem, apn, user, pass);
  // Setup a function to be called every second
  timer.setInterval(2000L, sendSensor);
}

void loop() {


  /* if (Serial.available() > 0)
    switch (Serial.read()) {
      case 'dustbin=>90':
        SendMessage();  //YOU CAN SEND MESSAGE FROM SIM TO THE MENTIONED PHONE NUMBER
        break;
      case 'r':
        RecieveMessage();  // RECEIVE MESSAGE FROM THE MENTIONED PHONE NUMBER TO SIM
        break;
      case 'c':
        callNumber();  // CALL
        break;
      case 'g':
        // textForSMS = textForSMS + "http://www.google.com/maps/place/" + mylati + "," + mylong ;   // GET GPS LOCATION
        break;
    }
  if (SerialAT.available() > 0)
    Serial.write(SerialAT.read());

*/

  Blynk.run();
  timer.run();
}

// H-Bridge Websites:     http://www.mouser.com/applications/industrial-motor-control-mosfets/?_cdc=0
//                        https://en.wikipedia.org/wiki/H_bridge
// Motor Driving Website: https://arduino-info.wikispaces.com/DC-Motors

#include <Adafruit_GPS.h> // Adafruit GPS library. Thales gave us an Adafruit Ultimate GPS Shield, so their library works perfectly.
#include <SoftwareSerial.h> // Software serial for GPS module. This is used to give us another serial port so we can communicate with the GPS shield and computer/radio at the same time.
#include <SharpIR.h> // Infrared distance sensor library. I think it's just a variable resistor, but regardless, the library works like a charm.
#include <SpritzCipher.h> // Encryption library.

// GPS Initialization
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false
boolean usingInterrupt = false;
void useInterrupt(boolean);
boolean printGPSData = true;

// Distance Sensor Initialization
SharpIR sensor(GP2YA41SK0F, A0);
boolean printDistanceData = false;
boolean tooClose = false;

// Misc.
int tempPin = 1;
int batteryPinOne = 2;
int batteryPinTwo = 3;
int batteryPinThree = 4;
int shortDataLength = 24;
int longDataLength = 50;
int direction = 0;
int speed = 0;

// Motor Controller
int enA = 5;
int in1 = 4;
int in2 = 7;
int in3 = 8;
int in4 = 9;

// Encryption
const byte testKey[64] = {0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63,0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63};

uint32_t timer = millis();

void setup()
{

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out

  Serial.begin(9600);
  Serial.println("Controller 0001");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA); // Request updates on antenna status, comment out to keep quiet
  useInterrupt(true);

  // Give the GPS module time to initialize itself
  delay(1000);
  
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

}

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void encrypt(const byte *msg, const byte *key, byte *omsg) {
  spritz_ctx s_ctx;
  omsg[0] = '\0';

  unsigned int msgLen = 0;
  byte keyLen = 64;

  for(int i = 0; i < 64; i++) {
    if(msg[i] == 0) {
      break;
    }
    msgLen++;
  }

  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, msg, msgLen, omsg);
}

void decrypt(const byte *msg, const byte *key, byte *omsg) {
  spritz_ctx s_ctx;

  unsigned int msgLen = 0;
  byte keyLen = 64;
  
  for(int i = 0; i < 64; i++) {
    if(msg[i] > 0) {
      msgLen++;
    }
  }
  msgLen = shortDataLength;
      
  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, msg, msgLen, omsg);
}

void loop()                     // run over and over again
{
  int distance = sensor.getDistance();
  int detectionDistance = 15;
  if(speed < 80)
    detectionDistance = 10;
  if(distance <= 10 && direction == 1) {
    analogWrite(enA, 0);
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
    digitalWrite(in3,LOW);
    digitalWrite(in4,LOW);
  }
  if(Serial.available()) {
    char c = Serial.read();
    if(c == 'S') {
      direction = 0;
      analogWrite(enA, 0);
      digitalWrite(in1,LOW);
      digitalWrite(in2,LOW);
      digitalWrite(in3,LOW);
      digitalWrite(in4,LOW);
    } else if(c == 'B') {
      direction = 2;
      speed = 100;
      if(Serial.available()) {
        speed = Serial.parseInt();
      }
      analogWrite(enA, speed);
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
    } else if(c == 'F') {
      direction = 1;
      speed = 100;
      if(Serial.available()) {
        speed = Serial.parseInt();
      }
      analogWrite(enA, speed);
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    } else if(c == 'L') {
      direction = 3;
      speed = 100;
      if(Serial.available()) {
        speed = Serial.parseInt();
      }
      analogWrite(enA, speed);
      digitalWrite(in1,HIGH);
      digitalWrite(in2,LOW);
      digitalWrite(in3,HIGH);
      digitalWrite(in4,LOW);
    } else if(c == 'R') {
      direction = 4;
      speed = 100;
      if(Serial.available()) {
        speed = Serial.parseInt();
      }
      analogWrite(enA, speed);
      digitalWrite(in1,LOW);
      digitalWrite(in2,HIGH);
      digitalWrite(in3,LOW);
      digitalWrite(in4,HIGH);
    }
  }
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    boolean full = false;
    char message[64];
    message[0] = '\0';
    strcat(message, "G");
    if (!GPS.parse(GPS.lastNMEA())) {   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    if (printGPSData) {
      if (GPS.fix) {
        full = true;
        char locationBuffer[7];
        dtostrf(GPS.latitudeDegrees, 7, 2, locationBuffer);
        strcat(message, locationBuffer);
        dtostrf(GPS.longitudeDegrees, 7, 2, locationBuffer);
        strcat(message, locationBuffer);
        char speedBuffer[5];
        dtostrf(GPS.speed, 5, 2, speedBuffer);
        strcat(message, speedBuffer);
        char altitudeBuffer[8];
        dtostrf(GPS.altitude, 8, 2, altitudeBuffer);
        strcat(message, altitudeBuffer);
        char satelliteBuffer[1];
        dtostrf(GPS.satellites, 1, 0, satelliteBuffer);
        strcat(message, satelliteBuffer);
        mySerial.print("A");
        mySerial.println(GPS.latitudeDegrees, 4);
        mySerial.print("O");
        mySerial.println(GPS.longitudeDegrees, 4);
      } else {
        strcat(message, "NF");
      }
    }
    char distanceBuffer[2];
    dtostrf(distance, 2, 0, distanceBuffer);
    strcat(message,distanceBuffer);
    if(distance <= 10) {
      strcat(message, "S");
    } else {
      strcat(message, "G");
    }
    double temperature = ((analogRead(tempPin)*5.0/1024.0)-0.5)*100.0;
    char temp[6];
    dtostrf(temperature, 6, 2, temp);
    strcat(message, temp);
    double batteryCellOne = analogRead(batteryPinOne)*5.0/1024.0;
    char batOne[4];
    dtostrf(batteryCellOne, 4, 2, batOne);
    strcat(message, batOne);
    double batteryCellTwo = analogRead(batteryPinTwo)*10.0/1024.0 - batteryCellOne;
    char batTwo[4];
    dtostrf(batteryCellTwo, 4, 2, batTwo);
    strcat(message, batTwo);
    double batteryCellThree = analogRead(batteryPinThree)*15.0/1024.0 - batteryCellTwo - batteryCellOne;
    char batThree[4];
    dtostrf(batteryCellThree, 4, 2, batThree);
    strcat(message, batThree);
    char encryptedMessage[64];
    encrypt(message, testKey, encryptedMessage);
    boolean found = false;
    if(full) {
      for(int i = longDataLength; i < 64; i++) {
        encryptedMessage[i] = '\0';
      }
    } else {
      for(int i = shortDataLength; i < 64; i++) {
        encryptedMessage[i] = '\0';
      }
    }
    Serial.println(encryptedMessage);
  }
  delay(100);
}

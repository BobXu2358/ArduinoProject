// H-Bridge Websites:     http://www.mouser.com/applications/industrial-motor-control-mosfets/?_cdc=0
//                        https://en.wikipedia.org/wiki/H_bridge
// Motor Driving Website: https://arduino-info.wikispaces.com/DC-Motors

#include <Adafruit_GPS.h> // Adafruit GPS library. Thales gave us an Adafruit Ultimate GPS Shield, so their library works perfectly.
#include <SoftwareSerial.h> // Software serial for GPS module. This is used to give us another serial port so we can communicate with the GPS shield and computer/radio at the same time.
#include <SharpIR.h> // Infrared distance sensor library. I think it's just a variable resistor, but regardless, the library works like a charm.

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

// Misc.
int redLEDPin = 4;
int greenLEDPin = 5;
int tempPin = 1;

void setup()
{
  // Initialize and turn on status LEDs
  pinMode(redLEDPin, OUTPUT);
  pinMode(greenLEDPin, OUTPUT);
  digitalWrite(redLEDPin, HIGH);
  digitalWrite(greenLEDPin, HIGH);

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);

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

  // Turn off status LEDs
  digitalWrite(redLEDPin, LOW);
  digitalWrite(greenLEDPin, LOW);
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

uint32_t timer = millis();
boolean tooClose = false;
void loop()                     // run over and over again
{
  int distance = sensor.getDistance();
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA())) {   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
    }
    if (printGPSData) {
      Serial.println();
      Serial.println();
      Serial.println();
      Serial.print("\nTime: ");
      Serial.print(GPS.hour, DEC); Serial.print(':');
      Serial.print(GPS.minute, DEC); Serial.print(':');
      Serial.print(GPS.seconds, DEC); Serial.print('.');
      Serial.print(GPS.milliseconds);
      Serial.println("GMT");
      Serial.print("Date: ");
      Serial.print(GPS.day, DEC); Serial.print('/');
      Serial.print(GPS.month, DEC); Serial.print("/20");
      Serial.println(GPS.year, DEC);
      if (GPS.fix) {
        Serial.print("Location: ");
        Serial.print(GPS.latitudeDegrees, 4);
        Serial.print(", ");
        Serial.println(GPS.longitudeDegrees, 4);

        Serial.print("Speed (knots): "); Serial.println(GPS.speed);
        Serial.print("Altitude: "); Serial.println(GPS.altitude);
        Serial.print("Satellites: ");
        if (GPS.satellites >= 5) {
          Serial.println("Good Lock");
        } else {
          Serial.println("Poor Reception");
        }
      } else {
        Serial.println("No Fix");
      }
      Serial.print("Infrared Distance: ");
      Serial.print(distance);
      if (distance <= 5) {
        Serial.println(" Too Close! Stop!");
      } else {
        Serial.println("");
      }
      Serial.print("Temperature (C): ");
      Serial.println(((analogRead(tempPin)*5.0/1024.0)-0.5)*100.0);
    }
  }

  // LED control basaed on distance sensor
  if (distance <= 5) {
    tooClose = true;
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
  } else {
    tooClose = false;
    digitalWrite(redLEDPin, LOW);
    digitalWrite(greenLEDPin, HIGH);
  }
}

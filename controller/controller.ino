#include <SpritzCipher.h> // Encryption library.
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2,3);

// Encryption
const byte testKey[64] = {0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63,0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63};

//Misc.
int shortDataLength = 24;
int longDataLength = 50;

void setup() {
  Serial.begin(115200);
  Serial.println("Message reciever.");
  mySerial.begin(9600);
}

void encrypt(const byte *msg, const byte *key, const byte *omsg) {
  spritz_ctx s_ctx;

  byte msgLen = 0;
  for(int i = 0; i < 64; i++) {
    if(msg[i] == 0)
      break;
    msgLen++;
  }
  msgLen++;
  byte keyLen = 64;

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
      
  spritz_setup(&s_ctx, key, keyLen);
  spritz_crypt(&s_ctx, msg, msgLen, omsg);
}

int time = millis();

void loop() {
  if(Serial.available()) {
    while(Serial.available()) {
      char c = Serial.read();
      mySerial.print(c);
    }
  }
  if(mySerial.available()) {
    time = millis();
    char buffer[64];
    int i = 0;
    char c = '\0';
    while(mySerial.available() && c != '\n') {
      c = mySerial.read();
      buffer[i] = c;
      i++;
    }
    buffer[i] = '\0';
    char decryptedMessage[64];
    decrypt(buffer, testKey, decryptedMessage);
    if(decryptedMessage[0] == 'G') {
      if(decryptedMessage[1] == 'N' && decryptedMessage[2] == 'F') {
        Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        for(int i = 0; i < shortDataLength; i++) {
          Serial.print(decryptedMessage[i]);
        }
        Serial.println();
        
        String distanceBuffer = "";
        distanceBuffer += decryptedMessage[3];
        distanceBuffer += decryptedMessage[4];
        int distance = distanceBuffer.toInt();
        Serial.print("Distance: ");
        Serial.println(distance);

        if(decryptedMessage[5] == 'G') {
          Serial.println("GO");
        } else {
          Serial.println("STOP");
        }

        String tempBuffer = "";
        tempBuffer += decryptedMessage[6];
        tempBuffer += decryptedMessage[7];
        tempBuffer += decryptedMessage[8];
        tempBuffer += decryptedMessage[9];
        tempBuffer += decryptedMessage[10];
        tempBuffer += decryptedMessage[11];
        float temp = tempBuffer.toFloat();
        Serial.print("Temperature: ");
        Serial.println(temp);

        String batOneBuffer = "";
        batOneBuffer += decryptedMessage[12];
        batOneBuffer += decryptedMessage[13];
        batOneBuffer += decryptedMessage[14];
        batOneBuffer += decryptedMessage[15];
        float batOne = batOneBuffer.toFloat();
        Serial.print("Battery Cell One Voltage: ");
        Serial.print(batOne);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batOne < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }

        String batTwoBuffer = "";
        batTwoBuffer += decryptedMessage[16];
        batTwoBuffer += decryptedMessage[17];
        batTwoBuffer += decryptedMessage[18];
        batTwoBuffer += decryptedMessage[19];
        float batTwo = batTwoBuffer.toFloat();
        Serial.print("Battery Cell Two Voltage: ");
        Serial.print(batTwo);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batTwo < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }

        String batThreeBuffer = "";
        batThreeBuffer += decryptedMessage[20];
        batThreeBuffer += decryptedMessage[21];
        batThreeBuffer += decryptedMessage[22];
        batThreeBuffer += decryptedMessage[23];
        float batThree = batThreeBuffer.toFloat();
        Serial.print("Battery Cell Three Voltage: ");
        Serial.print(batThree);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batThree < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }
      } else {
        Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
        for(int i = 0; i < longDataLength; i++) {
          Serial.print(decryptedMessage[i]);
        }
        Serial.println();

        Serial.println("GPS Fix");

        String latBuffer = "";
        latBuffer += decryptedMessage[1];
        latBuffer += decryptedMessage[2];
        latBuffer += decryptedMessage[3];
        latBuffer += decryptedMessage[4];
        latBuffer += decryptedMessage[5];
        latBuffer += decryptedMessage[6];
        latBuffer += decryptedMessage[7];
        float lat = latBuffer.toFloat();
        Serial.print("Lat: ");
        Serial.println(lat);

        String lonBuffer = "";
        lonBuffer += decryptedMessage[8];
        lonBuffer += decryptedMessage[9];
        lonBuffer += decryptedMessage[10];
        lonBuffer += decryptedMessage[11];
        lonBuffer += decryptedMessage[12];
        lonBuffer += decryptedMessage[13];
        lonBuffer += decryptedMessage[14];
        float lon = lonBuffer.toFloat();
        Serial.print("Lon: ");
        Serial.println(lon);

        String speedBuffer = "";
        speedBuffer += decryptedMessage[15];
        speedBuffer += decryptedMessage[16];
        speedBuffer += decryptedMessage[17];
        speedBuffer += decryptedMessage[18];
        speedBuffer += decryptedMessage[19];
        float speed = speedBuffer.toFloat();
        Serial.print("Speed: ");
        Serial.print(speed);
        Serial.println("kn");

        String altBuffer = "";
        altBuffer += decryptedMessage[20];
        altBuffer += decryptedMessage[21];
        altBuffer += decryptedMessage[22];
        altBuffer += decryptedMessage[23];
        altBuffer += decryptedMessage[24];
        altBuffer += decryptedMessage[25];
        altBuffer += decryptedMessage[26];
        altBuffer += decryptedMessage[27];
        float altitude = altBuffer.toFloat();
        Serial.print("Altitude: ");
        Serial.print(altitude);
        Serial.println("cm");

        String numOfSatBuffer = "";
        numOfSatBuffer += decryptedMessage[28];
        int numOfSat = numOfSatBuffer.toInt();
        Serial.print("Number of satellites: ");
        Serial.println(numOfSat);
        
        String distanceBuffer = "";
        distanceBuffer += decryptedMessage[29];
        distanceBuffer += decryptedMessage[30];
        int distance = distanceBuffer.toInt();
        Serial.print("Distance: ");
        Serial.println(distance);

        if(decryptedMessage[31] == 'G') {
          Serial.println("GO");
        } else {
          Serial.println("STOP");
        }

        String tempBuffer = "";
        tempBuffer += decryptedMessage[32];
        tempBuffer += decryptedMessage[33];
        tempBuffer += decryptedMessage[34];
        tempBuffer += decryptedMessage[35];
        tempBuffer += decryptedMessage[36];
        tempBuffer += decryptedMessage[37];
        float temp = tempBuffer.toFloat();
        Serial.print("Temperature: ");
        Serial.println(temp);

        String batOneBuffer = "";
        batOneBuffer += decryptedMessage[38];
        batOneBuffer += decryptedMessage[39];
        batOneBuffer += decryptedMessage[40];
        batOneBuffer += decryptedMessage[41];
        float batOne = batOneBuffer.toFloat();
        Serial.print("Battery Cell One Voltage: ");
        Serial.print(batOne);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batOne < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }

        String batTwoBuffer = "";
        batTwoBuffer += decryptedMessage[42];
        batTwoBuffer += decryptedMessage[43];
        batTwoBuffer += decryptedMessage[44];
        batTwoBuffer += decryptedMessage[45];
        float batTwo = batTwoBuffer.toFloat();
        Serial.print("Battery Cell Two Voltage: ");
        Serial.print(batTwo);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batTwo < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }

        String batThreeBuffer = "";
        batThreeBuffer += decryptedMessage[46];
        batThreeBuffer += decryptedMessage[47];
        batThreeBuffer += decryptedMessage[48];
        batThreeBuffer += decryptedMessage[49];
        float batThree = batThreeBuffer.toFloat();
        Serial.print("Battery Cell Three Voltage: ");
        Serial.print(batThree);
        Serial.println("V");
        Serial.print("\tStatus: ");
        if(batThree < 3.2) {
          Serial.println("Discharged");
        } else {
          Serial.println("Charged");
        }
      }
    }
  } else {
    int currentTime = millis();
    if(currentTime - time > 2000) {
      time = currentTime;
      Serial.println("\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n");
      Serial.println("No connection");
    }
  }
  delay(100);
}

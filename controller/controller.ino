#include <SpritzCipher.h> // Encryption library.
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2,3);

// Encryption
const byte testKey[64] = {0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63,0x43,0x5E,0xD6,0xD6,0x1C,0xA1,0x81,0x8B,0xBA,0x96,0xAB,0x98,0xB1,0xB6,0xC8,0x62,0x81,0x24,0xB6,0xF7,0xEB,0xA7,0x7A,0x7B,0xE4,0x24,0x57,0xF8,0x6C,0xB3,0x5A,0x63};

//Misc.
int shortDataLength = 24;
int longDataLength = 50;

void setup() {
  Serial.begin(9600);
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

void loop() {
  if(mySerial.available()) {
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
        for(int i = 0; i < shortDataLength; i++) {
          Serial.print(decryptedMessage[i]);
        }
        Serial.println();
      } else {
        for(int i = 0; i < longDataLength; i++) {
          Serial.print(decryptedMessage[i]);
        }
        Serial.println();
      }
    }
  }
  delay(100);
}

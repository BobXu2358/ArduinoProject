#include <SoftwareSerial.h>
#include <AESLib.h>

SoftwareSerial mySerial(2,3);

// Encryption
uint8_t key[] = {0,90,65,99,121,76,111,48,77,113,86,70,52,67,86,111,116,76,55,65,66,80,69,90,87,70,51,79,89,90,48,87,50};

//Misc.
int shortDataLength = 25;
int longDataLength = 51;

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
}

void loop() {
  if(Serial.available()) {
    while(Serial.available()) {
      char c = Serial.read();
      mySerial.print(c);
    }
  }
  if(mySerial.available()) {
    char inBuffer[64];
    int i = 0;
    char c = mySerial.read();
    while(c != '\n') {
      while(!mySerial.available());
      inBuffer[i] = c;
      i++;
      c = mySerial.read();
    }
    char decryptedMessage[64];
    for(int i = 0; i < 4; i++) {
      char messageChunk[16];
      for(int j = 0; j < 16; j++) {
        messageChunk[j] = inBuffer[16*i+j];
      }
      aes256_dec_single(key, messageChunk);
      for(int j = 0; j < 16; j++) {
          decryptedMessage[16*i+j] = messageChunk[j]; 
      }
    }
    if(decryptedMessage[0] == 'G') {
      int endPosition = String(decryptedMessage).indexOf('*');
      if(decryptedMessage[1] == 'N' && decryptedMessage[2] == 'F') {
        if(endPosition > 20 && endPosition < 30) {
          for(int i = 0; i < shortDataLength; i++) {
            Serial.print(decryptedMessage[i]);
          }
          Serial.println();
        }
      } else {
        if(endPosition > 45 && endPosition < 55) {
          for(int i = 0; i < longDataLength; i++) {
            Serial.print(decryptedMessage[i]);
          }
          Serial.println();
        }
      }
    }
  }
  delay(20);
}

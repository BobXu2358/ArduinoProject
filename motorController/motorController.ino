int enA = 6;
int in1 = 7;
int in2 = 8;

void setup() {
  pinMode(enA,OUTPUT);
  pinMode(in1,OUTPUT);
  pinMode(in2,OUTPUT);
  digitalWrite(in1,LOW);
  digitalWrite(in2,HIGH);
  Serial.begin(9600);
}

void loop() {
  int speed = 0;
  while(speed < 255) {
    speed += 5;
    Serial.println(speed);
    analogWrite(enA,speed);
    delay(100);
  }
  while(speed > 0) {
    speed -= 5;
    Serial.println(speed);
    analogWrite(enA,speed);
    delay(100);
  }
}

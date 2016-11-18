//DC Motor Control
//The DC motor will begin rotating left and right, and its speed will vary accordingly.

const int LF = 11;  // the pin controls left motor to spin forward
const int LB = 12;  // the pin controls left motor to spin backward
const int RF = 10;  // the pin controls right motor to spin forward
const int RB = 9;   // the pin controls right motor to spin backward
char byteIn;
int spd = 0;
void setup()
{
  //intialize the four pins as output
  Serial.begin(9600);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(RB, OUTPUT);
  Serial.println("Enter speed: (greater than 100 less than 255)\n");
  while(Serial.available() == 0) {}
  spd = Serial.parseInt();
  Serial.println(spd);
}

void loop()
{
   if(Serial.available() > 0) {
     byteIn = (char) Serial.read();
     Serial.println(byteIn);
     switch(byteIn) {
      case 'w':
        forward(spd);
        break;
      case 's':
        backward(spd);
        break;
      case 'a':
        leftTurn(spd);
        break;
      case 'd':
        rightTurn(spd);
        break;
      default:
        stop(500);
     }
   }
}

//The function to drive motor rotate clockwise
void forward(int spd) {
  analogWrite(LF, spd);
  analogWrite(LB, 0);
  analogWrite(RF, spd);
  analogWrite(RB, 0);
  stop(500);
}

//The function to drive motor rotate counterclockwise
void backward(int spd) {
  analogWrite(LF, 0);
  analogWrite(LB, spd);
  analogWrite(RF, 0);
  analogWrite(RB, spd);
  stop(500);
}

void leftTurn(int spd) {
  analogWrite(LF, spd);
  analogWrite(LB, 0);
  analogWrite(RF, 0);
  analogWrite(RB, spd);
  stop(500);
}

void rightTurn(int spd) {
  analogWrite(LF, 0);
  analogWrite(LB, spd);
  analogWrite(RF, spd);
  analogWrite(RB, 0);
  stop(500);
}

//after time ms stop the motors
void stop(int time) {
  delay(time);
  analogWrite(LF, 0);
  analogWrite(LB, 0);
  analogWrite(RF, 0);
  analogWrite(RB, 0);
}


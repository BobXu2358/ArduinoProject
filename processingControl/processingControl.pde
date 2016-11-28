import processing.serial.*;

Serial port;
int val;
//starting analog speed at 100, change this when needed
int spd = 100;
String out = "S";

void setup() {
   size(200, 200);
   //finds the first port
   String portName = Serial.list()[0];
   port = new Serial(this, portName, 9600);
}

void draw() {
   //moving forward
   if(keyPressed == true && (key == 'w' || key == 'W')) {
     //increment the speed so the car accelerate when key is pressed
       if(spd < 255)
         spd++;
       //generate serial command string
       out = "F" + spd;
   }
   //moving backward
   else if(keyPressed == true && (key == 's' || key == 'S')) {
       if(spd < 255)
         spd++;
       out = "B" + spd;
   }
   //turing left
   else if(keyPressed == true && (key == 'a' || key == 'A')) {
       if(spd < 255)
         spd++;
       out = "L" + spd;
   }
   //turn right
   else if(keyPressed == true && (key == 'd' || key == 'D')){
       if(spd < 255)
           spd++;
       out = "R" + spd;
   }
   //key released or other key pressed, stop the car
   else {
     //reset spd to initial value
     spd = 100;
     
     out = "S";
   }
   
   //just for debugging purpose to print out command string
   print(out + "\n");
   //output the stringto serial
   port.write(out);
   
   //change this value to change the acceleration
   delay(20);
}
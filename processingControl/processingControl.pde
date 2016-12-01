import processing.serial.*;

Serial port;
int val;
//starting analog speed at 100, change this when needed
int spd = 100;
String out = "S";

void setup() {
   //set up display window
   size(1200, 1200);
     background(255);
   //finds the first port, probably needs to modify if connected with mac or linux
   String portName = Serial.list()[0];
   println(portName);
   port = new Serial(this, portName, 115200);
}

void draw() {
   //clear previous info
   if(port.available() > 0)
     background(255);
   
   //constantly update information form serial input passed from arduino
   while(port.available() > 0) {
     //read string
       String in = port.readString();
       if(in != null)
         displayInput(in);
   }
   
   //delay(2000);
   
   //listen to keyboard press event and drive the car
   carControl();
}

void displayInput(String in) {
    println(in);
    //integer value stores vertical position for the text to display, is set to 30 every update cycle
    int vertPos = 30;
    //first check if the message is corrupted and display it 
    if(in.charAt(0) != 'G') {
        println("Messge is corrupted!");
        textSize(36);
        fill(50);
        text("Messge is corrupted!", 10, vertPos);
    }
    else {
      
        //check if information if GPS fix or not
        if(in.substring(1,3).equals("NF")) {
            //since right now i dont have the device by the time i write this code
            //so I will catch ArrayIndexOutOfBoundsException in case accessed invalid string index
            //this way the error will be handle gracefully(to console) and can be easilly located
            try {
                displayNF(in.substring(3), vertPos);
            }
            catch(StringIndexOutOfBoundsException e) {
                 println("Error displaying NF info");
            }
        }
        //GPS fix
        else {
            try {
                //display gps fix info
                
                //index 1-7 latitude
                drawText("Latitude", in.substring(1, 8), "", vertPos+=30);
                //index 8-14 longitude
                drawText("Longitude", in.substring(8, 15), "", vertPos+=30);
                //index 15-19 speed
                drawText("Speed(in knot)", in.substring(15, 20), "kn", vertPos+=30);
                //index 20-28 altitude
                drawText("Altitude", in.substring(20, 28), "cm", vertPos+=30);
                //index 29 number of satellites
                drawText("Number of satellites", in.substring(28, 29), "", vertPos+=30);
                
                //after index 30 the rest non-fix info
                displayNF(in.substring(29), vertPos);
            }
            catch(StringIndexOutOfBoundsException e) {
                 println("Error displaying FIX info");
            }
        }
    }
}

//display no GPS fix info
//to reduce unnecessary duplicate code it is written in a method
//param str: string contains NF info, do necessary modify prior calling
//param vertPos: current vertical postion
void displayNF(String str, int vertPos) {
    //see comments on drawText() method, vertPos is incremented by 30 after every piece of info
    //index 0-1 distance
    drawText("Distance from object in the front", str.substring(0, 2), "cm", vertPos+=30);
    //index 2 stop or going
    if(str.charAt(2) == 'S')
      drawText("Move forward", "disabled", "", vertPos+=30);
    else if(str.charAt(2) == 'G')
      drawText("Move forward", "enabled", "", vertPos+=30);
    
    //index 3-8 device tempture
    drawText("Device tempture(in Celsius)", str.substring(3, 9), "C", vertPos+=30);
    
    //after index 9: every 4 characters contains voltage, total 12 chars
    int index = 9;
    //read battery cell votage three times
    for(int i = 0; i < 3; i++)
        drawText("Battery cell #"+(i+1)+" voltage", str.substring(index, index+=4), "V", vertPos+=30);
}

//easy helper function to draw text to GUI display
//param cate: category of the info
//param value: value of the info
//param unit: unit of the value
//param vertPos: vertical position of the text
void drawText(String cate, String value, String unit, int vertPos) {
    //predifine text size to be 12 and color to be 50
    textSize(36);
    fill(50);
    
    //eliminate all white spaces in string
    value.replaceAll("\\s+","");
    //draw string
    text(cate + ": " + value + " " + unit, 10, vertPos);
 
}

void carControl() {
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
       //reset speed to initial value
       spd = 100;
       
       out = "S";
     }
     
     //just for debugging purpose to print out command string
     //print(out + "\n");
     //output the string to serial
     port.write(out);
     
     //change this value to change the acceleration
     delay(20);
}
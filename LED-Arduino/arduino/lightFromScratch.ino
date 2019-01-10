/*
 * Controlling LEDS for FRC....
 * 
 * Purpose: Updated 2019 light show for differnt field tasks.
 * 
 * Requires an Arduino with an Ethernet Shield or an Arduino with a built
 * in Ethernet port, such as the Leonardo or Yun.
*/
                                                    //Imports:
#include <SPI.h>                                    //SPI interface for interfacing with Ethernet Shield
#include <Ethernet.h>                               //Ethernet library for creating a client instance
#include <Adafruit_NeoPixel.h>                      //Adafruit's NeoPixel library for controlling NeoPixels


Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using 
  Adafruit_NeoPixel(120, 6, NEO_GRB + NEO_KHZ800);  //Arduino pin #6, and using the GRB format at 800KHZ bitstream
  
EthernetClient robotClient;                         //Defines a client to be used to connect to the Robo Rio
byte mac[] = {                                      //Creates a mac address for use in defining an Ethernet instance
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10,9,57,6);                            //Defines a static IP for the Arduino/Ethernet Shield
IPAddress robotIp(10,9,57,2);                       //Defines the robot's IP

int connectionTimer = 0;                            //Sets a connection timer to see if the program should reconnect to the RoboRio in case it becomes disconnected

char dataChar;                                      //Used for storing a character before inputing it into dataArray[]
String dataString = "";                             //Used for building a string to then splice into multiple parts to control the LEDs
String alliance = "invalid";                        //Sets a default alliance,
String gear = "false";                              //Gear state,
String climbing = "false";                          //Climbing state,
String xFinal = "2";                                //data for where the peg is via vision program,
String truexFinal = "center";
String inTeleOp = "false";  
String visionLocked = "false"; 
String outake = "succesful";                        //If the robot is in driver controlled mode
const int numberOfPixels=strip.numPixels;


int asciiArray[] = {                                //Please use the ASCII converter at https://www.arduino.cc/en/Reference/ASCIIchart
    32,44,45,48,49,50,51,52,53,54,55,56,            //to translate ASCII and add more possible characters to be readable
    57,97,98,99,100,101,102,103,104,105,            //Used for comparing data points to their ASCII counterparts
    106,107,108,109,110,111,112,113,114,
    115,116,117,118,119,120,121,122
};

char translationArray[] = {                         //Array housing characters to convert from ASCII to characters
  " ,-0123456789abcdefghijklmnopqrstuvwxyz"         //These are the usable characters for transferring data
};

int r;                                              //Alliance color red value
int b;                                              //Alliance color blue value


enum lightState{
  successfulClimb,
  visionLocked,
  duringMovement,
  succesfulOutake,
  still,
};

lightState robot = successfulClimb;
  
void setup() {                                      //Sets up constants before program begins     
  Ethernet.begin(mac,ip);                           //Initalizes an Ethernet instance
  Serial.begin(9600);                               //Initalizes serial communications to monitor data transfer
  robotClient.connect(robotIp, 5801);               //Connects the client instance to the robot's socket at 5801;
  strip.begin();                                    //Starts communication with the NeoPixel strip
}

void loop() {                                       //Ran indefinitly after setup()
  connectionTimer++;                                //Adds a count to the ConnectionTimer
  if(robotClient.available()){                      //Runs when bytes are available to read
    connectionTimer = 0;                            //Sets the connectionTimer countdown to zero
    dataString = "";                                //Resets our final data string
    while(robotClient.available()){                 //Processes data until program is out of readable bytes
      dataChar = asciiConvert(robotClient);         //Processes an ASCII byte into a readable character        
      dataString = dataString + dataChar;           //Combines the character with the full data string    
    }
    dataString.remove(0,2);                         //Removes two garbage characters from the start of the string

    alliance = dataString.substring(0,7);           //Grabs the expected location of various data, puts it in variables
    gear = dataString.substring(8,13);              
    climbing = dataString.substring(14,19);         
    xFinal = dataString.substring(20,21);
    inTeleOp = dataString.substring(22,26);
    if(xFinal == "1"){                              //If xFinal is equal to 1, it is to the left of the camera
      truexFinal = "left";
    }
    if(xFinal == "2"){                              //If xFinal is equal to 2, it is centered to the camera
      truexFinal = "center";
    }
    if(xFinal == "3"){                              //If xFinal is equal to 3, it is to the right of the camera
      truexFinal = "right";
    }
    Serial.println("Alliance?: " + alliance +       //Prints out the data above to the serial console
      ", Gear?: "  + gear + ", Climbing?: " + 
      climbing + ", xFinal?: " + truexFinal + ", " inTeleOp); 
  }
      
  //Below here is code to control the LED's from the data obtained above
  
  for(int i = 0; i < strip.numPixels(); i++){       //Resets the full LED strip...
    strip.setPixelColor(i,0,0,0);                   //...by setting each LED to black
  }
void reset(){
  strip.begin();
  strip.show(); //initialize all pixels to "off"
}

void ClimbLights() { 
  while(climbing == "true"){
    for(int i = 0; i < numberOfPixels; i++;){
      strip.setPixelColor(i,76,156,20);
      strip.show();
    }
    strip.reset();
  }
}

void visionLock(){
  while(visionLocked == "locked"){
    for(int i = 0; i < numberOfPixels; i++;){
      strip.setPixelColor(i,r,b,0);
    }
  }
}
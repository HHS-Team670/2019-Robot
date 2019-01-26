/*
 * Controlling LEDS for FRC....
 * 
 * Purpose: Updated 2019 light show for differnt field tasks.
 * 
 * Requires an Arduino with an Ethernet Shield or an Arduino with a built
 * in Ethernet port, such as the Leonardo or Yun.
 * 
 * @author arnav kulkarni, arnuv tandon
*/
#include <math.h>                                                   //Imports:
#include <SPI.h>                                    //SPI interface for interfacing with Ethernet Shield
#include <Ethernet.h>                               //Ethernet library for creating a client instance
#include <Adafruit_NeoPixel.h>                      //Adafruit library for led methods

Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using 
Adafruit_NeoPixel(15, 5, NEO_GRB + NEO_KHZ800); 

Adafruit_NeoPixel strip2 =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using 
Adafruit_NeoPixel(15, 7, NEO_GRB + NEO_KHZ800);//Arduino pin #6, and using the GRB format at 800KHZ bitstream

EthernetClient robotClient;                         //Defines a client to be used to connect to the Robo Rio
byte mac[] =                                        //Creates a mac address for use in defining an Ethernet instance
{                                                  
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(10,6,70,3);                            //Defines a static IP for the Arduino/Ethernet Shield
IPAddress robotIp(10,6,70,2);                       //Defines the robot's IP

int connectionTimer = 0;                          //Sets a connection timer to see if the program should reconnect to the RoboRio in case it becomes disconnected
char dataChar;                                    //Used for storing a character before inputing it into dataArray[]
const int numberOfPixels=strip.numPixels();
  
//string representations for alliances
const String blueAlliance = "1A";
const String redAlliance = "2A";

//string representations for robot states
const String climbing = "4R";
const String visionLock = "2R";
const String forwardDrive = "0R";
const String reverseDrive = "1R";
const String stillDrive = "3R";

const String runningAllianceColors = "0L";
const String solidGreen = "1L";
const String solidRed = "2L";
const String solidPurple = "3L";
const String climbingGreenLights = "4L";
const String Strobe = "5L";
const String randomStrobe = "6L";
const String bounceBackground = "7L";
const String cylonBounce = "8L";
const String rainbow = "9L";


//variables to store data recived from server
String dataString = "";                             //Used for building a string to then splice into multiple parts to control the LEDs
String stateData = stillDrive;                      //Sets default values for states
String allianceData = blueAlliance;
String lightShowData = runningAllianceColors;
//sets the entire strip to a specified color
void setStripColor(int r,int g, int b)
{
  for(int i = 0; i <= numberOfPixels; i++)
  {
    strip.setPixelColor(i,r,g,b);
  }
  strip.show();
}
void setRunningLights() {
  byte red=255;
  byte green=255;
  byte blue=0;
  int waveDelay=10;
  
  int Position=0;
  
   for(int j=0; j<strip.numPixels()*2; j++)
   {
      Position++; // = 0; //Position + Rate;
       for(int i=0; i<strip.numPixels(); i++) {
         // sine wave, 3 offset waves make a rainbow!
         //float level = sin(i+Position) * 127 + 128;
         //setPixel(i,level,0,0);
         //float level = sin(i+Position) * 127 + 128;
         strip.setPixelColor(i,((sin(i+Position) * 127 + 128)/255)*red,
                    ((sin(i+Position) * 127 + 128)/255)*green,
                   ((sin(i+Position) * 127 + 128)/255)*blue);
       }
      
       strip.show();
       delay(waveDelay);
   }
 }
void setRunningAllianceColors() //to be used when the robot is not moving, will flash alliance color
{
  if(stateData==stillDrive)
  {
    if(allianceData=blueAlliance)
    {                        
      setRunningLights();   
    } 
    else if(allianceData == redAlliance)
    {                  
      setRunningLights();    
    }
  }
}


void setSolidGreen() //to be used when the robot is moving forward, will display solid green
{
    setStripColor(0,255,0);
}

//reverse drive indicated by red
void setSolidRed() //to be used when the robot is moving in reverse, will display solid red
{
 
    setStripColor(255,0,0);

}

//solid blue color indicates vision lock
void setSolidPurple() //to be used when there is a vision lock, will display solid purple
{

    setStripColor(255,0,255);

}

//climbing green LEDs effect
void setClimbingGreenLights() //to be used when we are climbing, will display climbing green LEDS
{
    for(int i = 0; i <= numberOfPixels; i++)
    {                                 
      strip.setPixelColor(i,0,255,0);
      strip.show();
       delay(40);                              //Slows down the leds so we can see the effects    
    }
    for(int i=0;i<=numberOfPixels;i++){
         strip.setPixelColor(i,0,0,0);
         strip.show();
         delay(40);                            //Slows down the leds so we can see the effects
    }
  
}

void setStrobe(){
  byte red = 0;
  byte green = 0;
  byte blue = 255;

  int strobeCount=10;
  int flashDelay=20;
  int endPause=0;
  for(int j = 0; j < strobeCount; j++) {
    setStripColor(red,green,blue);
    strip.show();
    delay(flashDelay);
    reset();
    strip.show();
    delay(flashDelay);
  }
}
void setRandomStrobe(){
  
  int strobeCount= 10;
  int flashDelay = 10;
  int endPause = 0;
  for(int j = 0; j < strobeCount; j++) {
    for(int i=0;i<=strip.numPixels();i++){
      long randRed=random(1,255);
    long randGreen=random(1,255);
    long randBlue=random(1,255);
      strip.setPixelColor(i,randRed,randGreen,randBlue);
    }
    strip.show();
    strip.show();
    delay(flashDelay);
    reset();
    strip.show();
    delay(flashDelay);
  }
}
void setBounceBackground(){

byte red=255;
byte green=0;
byte blue=0;

int eyeSize=3;
int speedDelay=10;
int returnDelay=10;

  for(int i = 0; i < strip.numPixels()-eyeSize-2; i++) {
    setStripColor(255,255,255);
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= eyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+eyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(speedDelay);
  }

  delay(returnDelay);

  for(int i = strip.numPixels()-eyeSize-2; i > 0; i--) {
    setStripColor(255,255,255);
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= eyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+eyeSize+1, red/10, green/10, blue/10);
    strip.show();;
    delay(speedDelay);
  }
  
  delay(returnDelay);
}
void setCylonBounce(){
  byte red=255;
  byte green=0;
  byte blue=0;

  int eyeSize=3;
  int speedDelay=10;
  int returnDelay=10;
  for(int i = 0; i < strip2.numPixels()-eyeSize-2; i++) {
    reset();
    strip2.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= eyeSize; j++) {
      strip2.setPixelColor(i+j, red, green, blue); 
    }
    strip2.setPixelColor(i+eyeSize+1, red/10, green/10, blue/10);
    strip2.show();
    delay(speedDelay);
  }

  delay(returnDelay);

  for(int i = strip2.numPixels()-eyeSize-2; i > 0; i--) {
    reset();
    strip2.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= eyeSize; j++) {
      strip2.setPixelColor(i+j, red, green, blue); 
    }
    strip2.setPixelColor(i+eyeSize+1, red/10, green/10, blue/10);
    strip2.show();;
    delay(speedDelay);
  }
  
  delay(returnDelay);
}
void displayLightShow(){
    if(lightShowData=="0L"){
      setRunningAllianceColors();
    }else if(lightShowData=="1L"){
      setSolidGreen();
    }else if(lightShowData=="2L"){
      setSolidRed();
    }else if(lightShowData=="3L"){
      setSolidPurple();
    }else if(lightShowData=="4L"){
      setClimbingGreenLights();
    }else if(lightShowData=="5L"){
      setStrobe();
    }else if(lightShowData=="6L"){
      setRandomStrobe();
    }else if(lightShowData=="7L"){
      setBounceBackground();
    }else if(lightShowData=="8L"){
      setCylonBounce();
    }else if(lightShowData=="9L"){
      setRainbow();
    }
  
}

 
void setRainbow() {
  byte *c;
  int speedDelay=10;
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
        for (int i=0; i < strip2.numPixels(); i=i+3) {
          c = Wheel( (i+j) % 255);
          strip2.setPixelColor(i+q, *c, *(c+1), *(c+2));    //turn every third pixel on
        }
        strip2.show();
       
        delay(speedDelay);
       
        for (int i=0; i < strip2.numPixels(); i=i+3) {
          strip2.setPixelColor(i+q, 0,0,0);        //turn every third pixel off
        }
    }
  }
}

byte * Wheel(byte WheelPos) {
  static byte c[3];
  
  if(WheelPos < 85) {
   c[0]=WheelPos * 3;
   c[1]=255 - WheelPos * 3;
   c[2]=0;
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   c[0]=255 - WheelPos * 3;
   c[1]=0;
   c[2]=WheelPos * 3;
  } else {
   WheelPos -= 170;
   c[0]=0;
   c[1]=WheelPos * 3;
   c[2]=255 - WheelPos * 3;
  }
  

  return c;
}
//Sets the strip to black(no color)
void reset()
{
  setStripColor(0,0,0);
  strip.show();
}

void parseData()
{
  connectionTimer++;                                //Adds a count to the ConnectionTimer
  if(robotClient.available())                       //Runs when bytes are available to read
  {                                                 
    connectionTimer = 0;                            //Sets the connectionTimer countdown to zero
    dataString = "";                                //Resets our final data string
    while(robotClient.available())                  //Processes data until program is out of readable bytes
    {                
      char robotRead=(char)robotClient.read();      //Reads the sent data string
      dataString = dataString + robotRead;          //Combines the character with the full data string    
    }
  }
    //Parses dataString and receives corresponding values from Java program
    stateData = dataString.substring(0,2);           //Grabs the expected location of various data, puts it in variables
    allianceData = dataString.substring(2,4);   
    lightShowData = dataString.substring(4,6);           
  
}

void resetConnectionTimer()
{
  if(connectionTimer > 20)                          //About 1 second has passed since the last packet when one should come in every 1/4 of a second
  {                                                  
    connectionTimer = 0;                            //Resets the timer
    robotClient.stop();                             //Forces a socket disconnect from the RoboRio
    robotClient.connect(robotIp, 5801);             //Re-initalizes socket communication with the Rio
  }
}
  
void setup() 
{                                                   //Sets up constants before program begins     
  Ethernet.begin(mac,ip);                           //Initalizes an Ethernet instance
  Serial.begin(9600);                               //Initalizes serial communications to monitor data transfer
  robotClient.connect(robotIp, 5801);               //Connects the client instance to the robot's socket at 5801;
  strip.begin();    
  strip2.begin();
  //Starts communication with the NeoPixel strip
}

void loop()
{                                                   //Ran indefinitly after setup()
  
  parseData();
 strip2.setBrightness(100); 
if(stateData==stillDrive){
  displayLightShow();
} else if(stateData==forwardDrive){
  displayLightShow();
} else if(stateData==reverseDrive){
  displayLightShow();
} else if(stateData==visionLock){
  displayLightShow();
} else if(stateData==climbing){
  displayLightShow();
}

  //Below here is code to control the LED's from the data obtained above
 
    
  resetConnectionTimer();
}
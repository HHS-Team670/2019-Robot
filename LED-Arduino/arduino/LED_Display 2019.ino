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
Adafruit_NeoPixel(120, 6, NEO_GRB + NEO_KHZ800);    //Arduino pin #6, and using the GRB format at 800KHZ bitstream

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

//variables to store data recived from server
String dataString = "";                             //Used for building a string to then splice into multiple parts to control the LEDs
String stateData = stillDrive;                      //Sets default values for states
String allianceData = blueAlliance;
 
//sets the entire strip to a specified color
void setStripColor(int r,int g, int b)
{
  for(int i = 0; i <= numberOfPixels; i++)
  {
    strip.setPixelColor(i,r,g,b);
  }
  strip.show();
}

void Strobe(byte red, byte green, byte blue, int StrobeCount, int FlashDelay, int EndPause){
  for(int j = 0; j < StrobeCount; j++) {
    setStripColor(red,green,blue);
    strip.show();
    delay(FlashDelay);
    reset();
    strip.show();
    delay(FlashDelay);
  }
}
void randomStrobe(int StrobeCount, int FlashDelay, int EndPause){
  
  for(int j = 0; j < StrobeCount; j++) {
    for(int i=0;i<=strip.numPixels();i++){
      long randRed=random(1,255);
    long randGreen=random(1,255);
    long randBlue=random(1,255);
      strip.setPixelColor(i,randRed,randGreen,randBlue);
    }
    strip.show();
    strip.show();
    delay(FlashDelay);
    reset();
    strip.show();
    delay(FlashDelay);
  }
}
void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){

  for(int i = 0; i < strip.numPixels()-EyeSize-2; i++) {
    reset();
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for(int i = strip.numPixels()-EyeSize-2; i > 0; i--) {
    reset();
    strip.setPixelColor(i, red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue); 
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();;
    delay(SpeedDelay);
  }
  
  delay(ReturnDelay);
}

void RunningLights(byte red, byte green, byte blue, int WaveDelay) {
  
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
      delay(WaveDelay);
  }
}
//Sets the strip to black(no color)
void reset()
{
  setStripColor(0,0,0);
  strip.show();
}

  
void setup() 
{                                                   //Sets up constants before program begins     
  Ethernet.begin(mac,ip);                           //Initalizes an Ethernet instance
  Serial.begin(9600);                               //Initalizes serial communications to monitor data transfer
  robotClient.connect(robotIp, 5801);               //Connects the client instance to the robot's socket at 5801;
  strip.begin();                                    //Starts communication with the NeoPixel strip
}

void loop()
{                                                   //Ran indefinitly after setup()
reset();
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
  
  //Below here is code to control the LED's from the data obtained above
  strip.setBrightness(100); 
  if(stateData == stillDrive)
  {
    if(allianceData == blueAlliance)
    {                        
      setStripColor(0,0,255);                        //If the alliance is blue, set the base LED color to blue
    } 
    else if(allianceData==redAlliance)
    {                  
      setStripColor(255,0,0);                          //If the alliance is red, set the base LED color to red
    }
  }

  //climbing green LEDs effect
  if(stateData == climbing)
  {
    for(int i = 0; i <= numberOfPixels; i++)
    {
      delay(50);                                       //Slows down the leds so we can see the effects
      strip.setPixelColor(i,0,200,0);
      strip.show();
      
    }
    reset();   
  }

  //solid blue color indicates vision lock
  else if(stateData==visionLock)
  {
    setStripColor(10,67,35);
  }

  //forward drive is indicated by green
  else if(stateData==forwardDrive)
  {
    setStripColor(0,200,0);
  } 
    
  //reverse drive indicated by red
  else if(stateData==reverseDrive) 
  {
    setStripColor(255,0,0);
  }
        
  if(connectionTimer > 20)                          //About 1 second has passed since the last packet when one should come in every 1/4 of a second
  {                                                  
    connectionTimer = 0;                            //Resets the timer
    robotClient.stop();                             //Forces a socket disconnect from the RoboRio
    robotClient.connect(robotIp, 5801);             //Re-initalizes socket communication with the Rio
  }
}
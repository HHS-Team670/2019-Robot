/*
   Controlling LEDS for FRC....
   Purpose: Updated 2019 light show for differnt field tasks.
   Requires an Arduino with an Ethernet Shield or an Arduino with a built
   in Ethernet port, such as the Leonardo or Yun.
   @author arnav kulkarni, arnuv tandon
*/

//iterate through first quadrant
//add corrseponding quadrant values (for quadrant 1 add 0, for quadrant 2 add 15, for quadrant 3 add 30, and qudrant 4 add 45)
//then do strip.show() to show the values
//basically base off of the first quadrant and then add the multiple of 15
#include <math.h>                                                   //Imports:
#include <SPI.h>                                    //SPI interface for interfacing with Ethernet Shield
#include <Ethernet.h>                               //Ethernet library for creating a client instance
#include <Adafruit_NeoPixel.h>                      //Adafruit library for led methods

Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using
  Adafruit_NeoPixel(60, 7, NEO_GRB + NEO_KHZ800);


EthernetClient robotClient;                          //Defines a client to be used to connect to the Robo Rio
byte mac[] =                                         //Creates a mac address for use in defining an Ethernet instance
{
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

IPAddress ip(10, 6, 70, 3);                         //Defines a static IP for the Arduino/Ethernet Shield
IPAddress robotIp(10, 6, 70, 2);                    //Defines the robot's IP

int connectionTimer = 0;                          //Sets a connection timer to see if the program should reconnect to the RoboRio in case it becomes disconnected
char dataChar;                                    //Used for storing a character before inputing it into dataArray[]

//string representations for alliances
const String blueAlliance = "1A";
const String redAlliance = "2A";

//string representations for robot states
const String climbing = "4R";
const String visionLock = "2R";
const String forwardDrive = "0R";
const String reverseDrive = "1R";
const String stillDrive = "3R";

//const String runningAllianceColors = "0L";
//const String solidGreen = "1L";
//const String solidRed = "2L";
//const String solidPurple = "3L";
//const String climbingGreenLights = "4L";
//const String Strobe = "5L";
//const String randomStrobe = "6L";
//const String bounceBackground = "7L";
//const String cylonBounce = "8L";
//const String rainbow = "9L";


//variables to store data recived from server
String dataString = "";                             //Used for building a string to then splice into multiple parts to control the LEDs
String stateData = stillDrive;                      //Sets default values for states
String allianceData = blueAlliance;

//sets the entire strip to a specified color
void setStripColor(int r, int g, int b, int quadrant)
{
  for (int i = (strip.numPixels() / 4) * (quadrant - 1); i < (strip.numPixels() / 4) * quadrant; i++)
  {
    strip.setPixelColor(i, r, g, b);
  }
  strip.show();
}

void setRunningAllianceColors() {
  byte red = 255;
  byte blue = 0;
  byte green = 0;

  int Position = 0;

  for (int i = 0; i < strip.numPixels() * 2 ; i++)
  {
    parseData();
    if (stateData == stillDrive) {
      Position++; // = 0; //Position + Rate;
      int m, n;
      for (m = 0; m <= strip.numPixels(); m++) {

        if (allianceData == redAlliance) {
          strip.setPixelColor(m, ((sin(m + Position) * 127 + 128) / 255) * 255,
                              ((sin(m + Position) * 127 + 128) / 255) * 0,
                              ((sin(m + Position) * 127 + 128) / 255) * 0);
        }
        else if (allianceData == blueAlliance) {
          strip.setPixelColor(m, ((sin(m + Position) * 127 + 128) / 255) * 0,
                              ((sin(m + Position) * 127 + 128) / 255) * 0,
                              ((sin(m + Position) * 127 + 128) / 255) * 255);
        }
      }

      strip.show();
      delay(30);
    }
  }

}

void setSolidGreen(int quadrant) //to be used when the robot is moving forward, will display solid green
{
  setStripColor(0, 255, 0, quadrant);
}

//reverse drive indicated by red
void setSolidRed(int quadrant) //to be used when the robot is moving in reverse, will display solid red
{
  setStripColor(255, 0, 0, quadrant);
}

//solid blue color indicates vision lock
void setSolidPurple(int quadrant) //to be used when there is a vision lock, will display solid purple
{
  setStripColor(255, 0, 255, quadrant);
}
void setSolidWhite(int quadrant) {
  setStripColor(255, 255, 255, quadrant);
}

//climbing green LEDs effect

//climbing green LEDs effect for 2 quadrants
void setClimbingGreenLights() //to be used when we are climbing, will display climbing green LEDS
{

  for (int i = 0; i <= strip.numPixels(); i++)
  {
    strip.setPixelColor(i, 0, 255, 0);
    strip.show();
    delay(20);                              //Slows down the leds so we can see the effects
  }
  for (int i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, 0, 0, 0);
    strip.show();
    delay(20);                            //Slows down the leds so we can see the effects
  }

}

void strobe(int quadrant)
{
  byte red = 0;
  byte green = 0;
  byte blue = 255;

  int strobeCount = 10;
  int flashDelay = 20;
  int endPause = 0;
  if (stateData == forwardDrive) {
    parseData();
    for (int j = 0; j < strobeCount; j++)
    {
      setStripColor(red, green, blue, quadrant);
      strip.show();
      delay(flashDelay);

      reset(quadrant);
      strip.show();
      delay(flashDelay);
    }
  }
}
void setRandomStrobe()
{
  int strobeCount = 10;
  int flashDelay = 10;
  int endPause = 0;

  for (int j = 0; j < strobeCount; j++)
  {

    for (int i = 15; i <= 45; i++)
    {
      long randRed = random(1, 255);
      long randGreen = random(1, 255);
      long randBlue = random(1, 255);
      strip.setPixelColor(i, randRed, randGreen, randBlue);
    }
    strip.show();
    delay(flashDelay);

    reset(2);
    reset(3);
    strip.show();
    delay(flashDelay);

  }

}

void setCylonBounce()
{
  byte red = 0;
  byte green = 0;
  byte blue  = 255;
  int EyeSize = 3;
  int SpeedDelay = 30;
  int ReturnDelay = 30;

  for (int i = 15; i < 45 - EyeSize - 2; i++)
  {
    if (stateData = visionLock) {
      parseData();
      reset(2);
      reset(3);
      strip.setPixelColor(i, red / 10, green / 10, blue / 10);

      for (int j = 1; j <= EyeSize; j++)
      {
        strip.setPixelColor(i + j, red, green, blue);
      }
      strip.setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
      strip.show();
      delay(SpeedDelay);
    }
  }

  delay(ReturnDelay);

  for (int i = 45 - EyeSize - 2; i > 15; i--)
  {
    if (stateData = visionLock) {
      parseData();
      reset(2);
      reset(3);
      strip.setPixelColor(i, red / 10, green / 10, blue / 10);
      for (int j = 1; j <= EyeSize; j++)
      {
        strip.setPixelColor(i + j, red, green, blue);
      }
      strip.setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
      strip.show();
      delay(SpeedDelay);
    }
  }

  delay(ReturnDelay);
}
void setCylonBounce(int CVG)
{
  byte red = 255;
  byte green = 0;
  byte blue  = 0;
  int EyeSize = 3;
  int SpeedDelay = 30;
  int ReturnDelay = 30;

  for (int i = 15; i < 45 - EyeSize - 2; i++)
  {
    if (stateData = visionLock) {
      parseData();
      reset(2);
      reset(3);
      strip.setPixelColor(i, red / 10, green / 10, blue / 10);

      for (int j = 1; j <= EyeSize; j++)
      {
        strip.setPixelColor(i + j, red, green, blue);
      }
      strip.setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
      strip.show();
      delay(SpeedDelay);
    }
  }

  delay(ReturnDelay);

  for (int i = 45 - EyeSize - 2; i > 15; i--)
  {
    if (stateData = visionLock) {
      parseData();
      reset(2);
      reset(3);
      strip.setPixelColor(i, red / 10, green / 10, blue / 10);
      for (int j = 1; j <= EyeSize; j++)
      {
        strip.setPixelColor(i + j, red, green, blue);
      }
      strip.setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
      strip.show();
      delay(SpeedDelay);
    }
  }

  delay(ReturnDelay);
}

void meteorRain()
{

  byte red = 255;
  byte blue = 255;
  byte green = 0;

  int meteorSize = 5;
  int meteorTrailDecay = 10;
  boolean meteorRandomDecay = true;
  int speedDelay = 30;


  reset(2);
  reset(3);

  for (int i = 0; i < 2 * (strip.numPixels()); i++)
  {
    // fade brightness all LEDs one step
    for (int j = 15; j < 45; j++)
    {
      if ( (!meteorRandomDecay) || (random(10) > 5) )
      {
        fadeToBlack(j, meteorTrailDecay);
      }
    }

    // draw meteor
    for (int j = 0; j < meteorSize; j++)
    {
      if ( ( i - j < strip.numPixels()) && (i - j >= 0) )
      {
        strip.setPixelColor(i - j, red, green, blue);
      }
    }

    strip.show();
    delay(speedDelay);
  }
}

void fadeToBlack(int ledNo, byte fadeValue)
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  uint32_t oldColor;
  uint8_t r, g, b;
  int value;

  oldColor = strip.getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
  g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
  b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);

  strip.setPixelColor(ledNo, r, g, b);
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[ledNo].fadeToBlackBy( fadeValue );
#endif
}


void setRainbow()
{
  byte *c;
  int speedDelay = 10;
  for (int j = 0; j < 256; j++)                                // cycle all 256 colors in the wheel
  {
    parseData();
    if (stateData == reverseDrive) {
      for (int q = 0; q < 3; q++)
      {
        for (int i = 15; i <= 43 ; i = i + 3)
        {
          c = Wheel( (i + j) % 255);
          strip.setPixelColor(i + q, *c, *(c + 1), *(c + 2)); //turn every third pixel on
        }
        strip.show();

        delay(speedDelay);

        for (int i = 15; i < 45; i = i + 3)
        {
          strip.setPixelColor(i + q, 0, 0, 0);    //turn every third pixel off
        }
      }
    }
  }
}
byte * Wheel(byte WheelPos)
{
  static byte c[3];

  if (WheelPos < 85)
  {
    c[0] = WheelPos * 3;
    c[1] = 255 - WheelPos * 3;
    c[2] = 0;
  }
  else if (WheelPos < 170)
  {
    WheelPos -= 85;
    c[0] = 255 - WheelPos * 3;
    c[1] = 0;
    c[2] = WheelPos * 3;
  }
  else
  {
    WheelPos -= 170;
    c[0] = 0;
    c[1] = WheelPos * 3;
    c[2] = 255 - WheelPos * 3;
  }


  return c;
}



//Sets the strip to black(no color)
void reset(int quadrant)
{
  setStripColor(0, 0, 0, quadrant);
  strip.show();
}

void parseData()
{
  connectionTimer++;                                //Adds a count to the ConnectionTimer
  if (robotClient.available())                      //Runs when bytes are available to read
  {
    connectionTimer = 0;                            //Sets the connectionTimer countdown to zero
    dataString = "";                                //Resets our final data string
    while (robotClient.available())                 //Processes data until program is out of readable bytes
    {
      char robotRead = (char)robotClient.read();    //Reads the sent data string
      dataString = dataString + robotRead;          //Combines the character with the full data string
    }
  }
  //Parses dataString and receives corresponding values from Java program
  allianceData = dataString.substring(0, 2);
  Serial.print(allianceData);
  //Grabs the expected location of various data, puts it in variables
  stateData = dataString.substring(2, 4);
  Serial.println(stateData);
}

void resetConnectionTimer()
{
  if (connectionTimer > 20)                         //About 1 second has passed since the last packet when one should come in every 1/4 of a second
  {
    connectionTimer = 0;                            //Resets the timer
    robotClient.stop();                             //Forces a socket disconnect from the RoboRio
    robotClient.connect(robotIp, 5801);             //Re-initalizes socket communication with the Rio
  }
}

void setup()
{ //Sets up constants before program begins
  Ethernet.begin(mac, ip);                          //Initalizes an Ethernet instance
  Serial.begin(9600);                               //Initalizes serial communications to monitor data transfer
  robotClient.connect(robotIp, 5801);               //Connects the client instance to the robot's socket at 5801;
  strip.begin();
  //Starts communication with the NeoPixel strip
}

void loop()
{ //Ran indefinitly after setup()

  parseData();
  Serial.println(stateData + "STATE");
  Serial.println(allianceData + "ALLLIANCE");

  strip.setBrightness(255);

  if (stateData.equals(forwardDrive))
  {
    setSolidGreen(1);
    setSolidGreen(4);
    setCylonBounce(1);
  }
  else if (stateData.equals(reverseDrive))
  {
    setSolidRed(1);
    setSolidRed(4);
    setRainbow();
  }
  else if (stateData.equals(visionLock))
  {
    setSolidPurple(1);
    setSolidPurple(4);
    setCylonBounce();
  }
  else if (stateData.equals(climbing))
  {
    setClimbingGreenLights();
  }
  else
  {
    setRunningAllianceColors();
  }

  resetConnectionTimer();
}
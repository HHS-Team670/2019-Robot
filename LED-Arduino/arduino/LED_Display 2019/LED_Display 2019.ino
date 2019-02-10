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
  Adafruit_NeoPixel(60, 5, NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel strip2 =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using
  Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);    //Arduino pin #6, and using the GRB format at 800KHZ bitstream

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

//sets the entire strip to a specified color
void setStripColor(int r, int g, int b, Adafruit_NeoPixel *stripptr, int quadrant)
{
  for (int i = (stripptr->numPixels() / 4) * (quadrant - 1); i < (stripptr->numPixels() / 4) * quadrant; i++)
  {
    stripptr->setPixelColor(i, r, g, b);

  }
  stripptr->show();
}

void setRunningAllianceColors(Adafruit_NeoPixel*stripptr) {
  byte red = 255;
  byte blue = 0;
  byte green = 0;

  int Position = 0;

  for (int i = 0; i < strip.numPixels() * 2 ; i++)
  {
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

void setSolidGreen(Adafruit_NeoPixel *stripptr, int quadrant) //to be used when the robot is moving forward, will display solid green
{
  setStripColor(0, 255, 0, stripptr, quadrant);
}

//reverse drive indicated by red
void setSolidRed(Adafruit_NeoPixel *stripptr, int quadrant) //to be used when the robot is moving in reverse, will display solid red
{
  setStripColor(255, 0, 0, stripptr, quadrant);
}

//solid blue color indicates vision lock
void setSolidPurple(Adafruit_NeoPixel *stripptr, int quadrant) //to be used when there is a vision lock, will display solid purple
{
  setStripColor(255, 0, 255, stripptr, quadrant);
}

//climbing green LEDs effect

//climbing green LEDs effect for 2 quadrants
void setClimbingGreenLights(Adafruit_NeoPixel *stripptr) //to be used when we are climbing, will display climbing green LEDS
{
  for (int i = 0; i <= strip.numPixels(); i++)
  {
    stripptr->setPixelColor(i, 0, 255, 0);
    stripptr->show();
    delay(20);                              //Slows down the leds so we can see the effects
  }
  for (int i = 0; i < strip.numPixels(); i++)
  {
    stripptr->setPixelColor(i, 0, 0, 0);
    stripptr->show();
    delay(20);                            //Slows down the leds so we can see the effects
  }
}

void strobe(Adafruit_NeoPixel *stripptr, int quadrant)
{
  byte red = 0;
  byte green = 0;
  byte blue = 255;

  int strobeCount = 10;
  int flashDelay = 20;
  int endPause = 0;

  for (int j = 0; j < strobeCount; j++)
  {
    setStripColor(red, green, blue, stripptr, quadrant);
    stripptr->show();
    delay(flashDelay);

    reset(stripptr, quadrant);
    stripptr->show();
    delay(flashDelay);
  }
}
void setRandomStrobe(Adafruit_NeoPixel *stripptr)
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
      stripptr->setPixelColor(i, randRed, randGreen, randBlue);
    }
    stripptr->show();
    delay(flashDelay);

    reset(stripptr, 2);
    reset(stripptr, 3);
    stripptr->show();
    delay(flashDelay);
  }
}

void setCylonBounce(Adafruit_NeoPixel *stripptr)
{
  byte red = 0;
  byte green = 0;
  byte blue  = 255;
  int EyeSize = 3;
  int SpeedDelay = 30;
  int ReturnDelay = 30;

  for (int i = 15; i < 45 - EyeSize - 2; i++)
  {
    reset(stripptr, 2);
    reset(stripptr, 3);
    stripptr->setPixelColor(i, red / 10, green / 10, blue / 10);

    for (int j = 1; j <= EyeSize; j++)
    {
      stripptr->setPixelColor(i + j, red, green, blue);
    }
    stripptr->setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    stripptr->show();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);

  for (int i = 45 - EyeSize - 2; i > 15; i--)
  {
    reset(stripptr, 2);
    reset(stripptr, 3);
    stripptr->setPixelColor(i, red / 10, green / 10, blue / 10);
    for (int j = 1; j <= EyeSize; j++)
    {
      stripptr->setPixelColor(i + j, red, green, blue);
    }
    stripptr->setPixelColor(i + EyeSize + 1, red / 10, green / 10, blue / 10);
    stripptr->show();
    delay(SpeedDelay);
  }

  delay(ReturnDelay);
}
void bouncingBalls(Adafruit_NeoPixel *stripptr, int quadrant)
{
  byte red  = 255;
  byte blue = 255;
  byte green = 0;
  int ballCount = 3;
  float Gravity = -9.81;
  int StartHeight = 1;

  float Height[ballCount];
  float ImpactVelocityStart = sqrt( -2 * Gravity * StartHeight );
  float ImpactVelocity[ballCount];
  float TimeSinceLastBounce[ballCount];
  int   Position[ballCount];
  long  ClockTimeSinceLastBounce[ballCount];
  float Dampening[ballCount];

  for (int i = 0 ; i < ballCount ; i++)
  {
    ClockTimeSinceLastBounce[i] = millis();
    Height[i] = StartHeight;
    Position[i] = 0;
    ImpactVelocity[i] = ImpactVelocityStart;
    TimeSinceLastBounce[i] = 0;
    Dampening[i] = 0.90 - float(i) / pow(ballCount, 2);
  }

  while (true)
  {
    for (int i = 0 ; i < ballCount ; i++)
    {
      TimeSinceLastBounce[i] =  millis() - ClockTimeSinceLastBounce[i];
      Height[i] = 0.5 * Gravity * pow( TimeSinceLastBounce[i] / 1000 , 2.0 ) + ImpactVelocity[i] * TimeSinceLastBounce[i] / 1000;

      if ( Height[i] < 0 )
      {
        Height[i] = 0;
        ImpactVelocity[i] = Dampening[i] * ImpactVelocity[i];
        ClockTimeSinceLastBounce[i] = millis();

        if ( ImpactVelocity[i] < 0.01 )
        {
          ImpactVelocity[i] = ImpactVelocityStart;
        }
      }
      Position[i] = round( Height[i] * (stripptr->numPixels() - 1) / StartHeight);
    }

    for (int i = 0 ; i < ballCount ; i++)
    {
      stripptr->setPixelColor(Position[i], red, green, blue);
    }

    stripptr->show();
    reset(stripptr, quadrant);
  }
}
void meteorRain(Adafruit_NeoPixel *stripptr)
{

  byte red = 255;
  byte blue = 255;
  byte green = 0;

  int meteorSize = 5;
  int meteorTrailDecay = 10;
  boolean meteorRandomDecay = true;
  int speedDelay = 30;


  reset(stripptr, 2);
  reset(stripptr, 3);

  for (int i = 0; i < 2 * (stripptr->numPixels()); i++)
  {
    // fade brightness all LEDs one step
    for (int j = 15; j < 45; j++)
    {
      if ( (!meteorRandomDecay) || (random(10) > 5) )
      {
        fadeToBlack(j, meteorTrailDecay, stripptr);
      }
    }

    // draw meteor
    for (int j = 0; j < meteorSize; j++)
    {
      if ( ( i - j < stripptr->numPixels()) && (i - j >= 0) )
      {
        stripptr->setPixelColor(i - j, red, green, blue);
      }
    }

    stripptr->show();
    delay(speedDelay);
  }
}

void fadeToBlack(int ledNo, byte fadeValue, Adafruit_NeoPixel *stripptr)
{
#ifdef ADAFRUIT_NEOPIXEL_H
  // NeoPixel
  uint32_t oldColor;
  uint8_t r, g, b;
  int value;

  oldColor = stripptr->getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r = (r <= 10) ? 0 : (int) r - (r * fadeValue / 256);
  g = (g <= 10) ? 0 : (int) g - (g * fadeValue / 256);
  b = (b <= 10) ? 0 : (int) b - (b * fadeValue / 256);

  stripptr->setPixelColor(ledNo, r, g, b);
#endif
#ifndef ADAFRUIT_NEOPIXEL_H
  // FastLED
  leds[ledNo].fadeToBlackBy( fadeValue );
#endif
}


void setRainbow(Adafruit_NeoPixel *stripptr)
{
  byte *c;
  int speedDelay = 10;
  for (int j = 0; j < 256; j++)                                // cycle all 256 colors in the wheel
  {
    // UNCOMMENT BEFORE RUNNING WITH ROBORIO
    parseData();
    while (stateData == reverseDrive) {
      for (int q = 0; q < 3; q++)
      {
        for (int i = 15; i <= 43 ; i = i + 3)
        {
          c = Wheel( (i + j) % 255);
          stripptr->setPixelColor(i + q, *c, *(c + 1), *(c + 2)); //turn every third pixel on
        }
        stripptr->show();

        delay(speedDelay);

        for (int i = 15; i < 45; i = i + 3)
        {
          stripptr->setPixelColor(i + q, 0, 0, 0);    //turn every third pixel off
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
void reset(Adafruit_NeoPixel *stripptr, int quadrant)
{
  setStripColor(0, 0, 0, stripptr, quadrant);
  stripptr->show();
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
  stateData = dataString.substring(0, 2);          //Grabs the expected location of various data, puts it in variables
  allianceData = dataString.substring(2, 4);
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
  strip2.begin();
  //Starts communication with the NeoPixel strip
}

void loop()
{ //Ran indefinitly after setup()


  parseData();
  strip.setBrightness(255);
  strip2.setBrightness(255);

  if (stateData == stillDrive)
  {
    setRunningAllianceColors(&strip);
  }
  else if (stateData == forwardDrive)
  {
    setSolidGreen(&strip, 1);
    setSolidGreen(&strip, 4);
    setRandomStrobe(&strip);
  }
  else if (stateData == reverseDrive)
  {
    setSolidRed(&strip, 1);
    setSolidRed(&strip, 4);
    setRainbow(&strip);
  }
  else if (stateData == visionLock)
  {
    setSolidPurple(&strip, 1);
    setSolidPurple(&strip, 4);
    setCylonBounce(&strip);
  }
  else if (stateData == climbing)
  {
    setClimbingGreenLights(&strip);
  }

  resetConnectionTimer();
}

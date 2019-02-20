/*
  Includes both strobe and random_strobe animations (only difference is colors)
*/

#include <Adafruit_NeoPixel.h>

Adafruit_NeoPixel strip =                          
  Adafruit_NeoPixel(60, 7, NEO_GRB + NEO_KHZ800);
  
void strobe(int red, int green, int blue)
{
  int strobeCount = 10;
  int flashDelay = 20;
  int endPause = 0;
  if (stateData == forwardDrive) {
    parseData();
    for (int j = 0; j < strobeCount; j++)
    {
      setStripColor(red, green, blue);
      strip.show();
      delay(flashDelay);

      setStripColor(0, 0, 0);
      strip.show();
      delay(flashDelay);
    }
  }
}

void randomStrobe()
{
  int strobeCount = 10;
  int flashDelay = 10;
  int endPause = 0;

  for (int j = 0; j < strobeCount; j++)
  {
    for (int i = 0; i <= strip.numPixels(); i++)
    {
      long randRed = random(1, 255);
      long randGreen = random(1, 255);
      long randBlue = random(1, 255);
      strip.setPixelColor(i, randRed, randGreen, randBlue);
    }
    strip.show();
    delay(flashDelay);

    strip.setStripColor(0, 0, 0);
    strip.show();
    delay(flashDelay);

  }
}

void setStripColor(int r, int g, int b)
{
  for (int i = 0; i < strip.numPixels(); i++)
  {
    strip.setPixelColor(i, r, g, b);
  }
  strip.show();
}

void setup()
{ 
  strip.begin();
}

void loop()
{ 
    strip.setBrightness(255);
    strobe(255, 0, 0); //strobe animation with red color
    //randomStrobe();
}

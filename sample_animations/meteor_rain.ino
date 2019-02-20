
#include <Adafruit_NeoPixel.h>                      //Adafruit library for led methods

Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using
  Adafruit_NeoPixel(60, 7, NEO_GRB + NEO_KHZ800);

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

void setup()
{ 
  strip.begin();
}

void loop()
{ 
    strip.setBrightness(255);
    meteorRain();
}

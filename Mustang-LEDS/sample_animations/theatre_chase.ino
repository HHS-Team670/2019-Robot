/*
Theatre chase animation for Neopixel strip
*/

#include <Adafruit_NeoPixel.h>                      //Adafruit library for led methods

Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 60 LEDs, using
  Adafruit_NeoPixel(60, 7, NEO_GRB + NEO_KHZ800);

void theaterChase(uint32_t c, uint8_t wait) {
  for (int j = 0; j < 10; j++) { //do 10 cycles of chasing
    for (int q = 0; q < 3; q++) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, c);  //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
        strip.setPixelColor(i + q, 0);      //turn every third pixel off
      }
    }
  }
}


void setup()
{ 
  strip.begin();
}

void loop()
{ 
    strip.setBrightness(255);
    theaterChase(strip.Color(0, 255, 255), 50); // Displays aqua theatre chase animation
}

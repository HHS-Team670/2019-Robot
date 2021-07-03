/*
 * Controlling LEDS for FRC....
 * 
 * Purpose: To recive data from a RoboRio that contains circumstantial data
 * to control NeoPixel LED strips to provide driver feedback and "bling".
 * 
 * Requires an Arduino with an Ethernet Shield or an Arduino with a built
 * in Ethernet port, such as the Leonardo or Yun.
*/
                                                    //Imports:
#include <SPI.h>                                    //SPI interface for interfacing with Ethernet Shield
#include <Ethernet.h>                               //Ethernet library for creating a client instance
#include <Adafruit_NeoPixel.h>                      //Adafruit's NeoPixel library for controlling NeoPixels

Adafruit_NeoPixel strip =                           //Defines an Adafruit Neopixel strip, containing 120 LEDs, using 
  Adafruit_NeoPixel(16, 6, NEO_GRB + NEO_KHZ800);  //Arduino pin #6, and using the GRB format at 800KHZ bitstream
  
EthernetClient robotClient;                         //Defines a client to be used to connect to the Robo Rio
byte mac[] = {                                      //Creates a mac address for use in defining an Ethernet instance
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
IPAddress ip(10,6,70,3);                            //Defines a static IP for the Arduino/Ethernet Shield
IPAddress robotIp(10,6,70,2);                       //Defines the robot's IP

int connectionTimer = 0;                            //Sets a connection timer to see if the program should reconnect to the RoboRio in case it becomes disconnected

char dataChar;                                      //Used for storing a character before inputing it into dataArray[]
String dataString = "";                             //Used for building a string to then splice into multiple parts to control the LEDs
String alliance = "invalid";                        //Sets a default alliance,
String gear = "false";                              //Gear state,
String climbing = "false";                          //Climbing state,
String xFinal = "2";                                //data for where the peg is via vision program,
String truexFinal = "center";
String inTeleOp = "false";                           //If the robot is in driver controlled mode
int chaseOffset = 0;
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
  
  if(alliance == "blue   "){                        //If the alliance is blue, set the base LED color to blue
    r = 0;
    b = 255;
  }
  if(alliance == "red    "){                        //If the alliance is red, set the base LED color to red
    r = 255;
    b = 0;
  }
  if(alliance == "invalid"){                        //If no alliance is specified, set the base LED color to purple
    r = 255;
    b = 255;
  }
  
  if(climbing == "true "){                          //If the robot is climbing, make the LEDs climb upward
    for(int i = chaseOffset; i < 26; i = i+3){      //Lights up one side with Leds moving down the strip in yellow
      if(i >! 26){
        strip.setPixelColor(i,255,255,0);
      }
    }

    for(int i = 25; i < 60; i++){                   //Lights up the top with our alliance color
      strip.setPixelColor(i,r,0,b);
    }
    
    for(int i = -chaseOffset; i < 120; i = i+3){    //Lights the other side with Leds moving up the strip in yellow
      if(i > 59 && i < 120){
        strip.setPixelColor(i,255,255,0);
      }
    }
   
  }else{                                            //If the robot is not climbing, perform all our other LED actions
    if(gear == "true "){                            //If the robot has a gear, have the top part of our strip illuminate green
      for(int i = 26; i < 60; i++){                 //and perform an animation depending on the location of the peg.
        strip.setPixelColor(i,255,0,0);             
      }                       
      for(int i = 0; i < 26; i++){       
        strip.setPixelColor(i,r,0,b);               
      }
      for(int i = 60; i < 119; i++){       
        strip.setPixelColor(i,r,0,b);               
      }      
    }else{
      for(int i = 0; i < strip.numPixels(); i++){   //Sets the full LED strip to our alliance color
        strip.setPixelColor(i,r,0,b);                       
      }
    }
  } 
  strip.show();                                     //Update the LED strip with colors set above

  if(chaseOffset == 2){ 
    chaseOffset = 0;                               
  }else{
    chaseOffset++;
  }
  delay(50);

  if(connectionTimer > 20){                         //About 1 second has passed since the last packet when one should come in every 1/4 of a second
    connectionTimer = 0;                            //Resets the timer
    robotClient.stop();                             //Forces a socket disconnect from the RoboRio
    robotClient.connect(robotIp, 5801);             //Re-initalizes socket communication with the Rio
  }
}

char asciiConvert(EthernetClient client){           //Changes an ASCII byte from a socket to a string

  int data = client.read();                         //Reads the socket for data from server
  int dataPoint = -666;                             //Defaults the point that the recived bytes equal a char in asciiArray[]
  for(int i=0; i < 37 -1; i++){                     //Runs the length of asciiArray[]
    if(asciiArray[i] == data){                      //If the data recived equals a point in asciiArray[]...
      dataPoint = i;                                //...the program sets dataPoint to the point of equality
    }
  }

  char finalData = translationArray[dataPoint];     //Sets the final character 
  if(dataPoint != -666){                            //If the data point is not null...
    return finalData;                               //Returns the transcribed ASCII character
  }else{
    char defaultchar[] = {"E"};                     //Sets a default char if there is an error
    return defaultchar[0];                          //Returns the default error char
  }
}

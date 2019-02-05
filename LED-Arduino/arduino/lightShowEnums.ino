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
#include <Adafruit_NeoPixel.h> 

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
// String xFinal = "2";                                //data for where the peg is via vision program,
// String truexFinal = "center";
// String alliance = "Invalid";
// String climbing = "false  ";
// String forwardDrive="false  ";
// String reverseDrive="false  ";
// String still="false  ";
// String visionLock="false  ";
//enum robot state declarations




String stateData;
String allianceData;
String xFinalData;


int numberOfPixels=strip.numPixels();

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

void loop() {
   for(int i = 0; i < numberOfPixels; i++){
        strip.setPixelColor(i,255,255,255);
   }
        strip.show();    
           //Ran indefinitly after setup()
  connectionTimer++;                                //Adds a count to the ConnectionTimer
  
  if(robotClient.available()){                      //Runs when bytes are available to read
   for(int i = 0; i < numberOfPixels; i++){
        strip.setPixelColor(i,255,0,0);}//green
        strip.show();
    connectionTimer = 0;                            //Sets the connectionTimer countdown to zero
    dataString = "";                                //Resets our final data string
    while(robotClient.available()){                 //Processes data until program is out of readable bytes
      dataChar = asciiConvert(robotClient);         //Processes an ASCII byte into a readable character        
      dataString = dataString + dataChar;           //Combines the character with the full data string    
    }
    //parses dataString and receives corresponding values from Java program
    dataString.remove(0,2);                         //Removes two garbage characters from the start of the string
    stateData = dataString.substring(0,1);           //Grabs the expected location of various data, puts it in variables
    allianceData = dataString.substring(2,3);              
    xFinalData=dataString.substring(4,5);

    
    
    if(xFinalData=="1X"){                              //If xFinal is equal to 1, it is to the left of the camera
      for(int i = 0; i < strip.numPixels(); i++){       //Resets the full LED strip...
    strip.setPixelColor(i,244, 100, 240);                  
  }
    } else if(xFinalData=="2X"){                              //If xFinal is equal to 2, it is centered to the camera
    // 25 25 45
      for(int i = 0; i < strip.numPixels(); i++){       //Resets the full LED strip...
    strip.setPixelColor(i,18 ,226,18);                   
  }
    } else if(xFinalData=="3X"){                              //If xFinal is equal to 3, it is to the right of the camera
    // 10 255 100
      for(int i = 0; i < strip.numPixels(); i++){       //Resets the full LED strip...
    strip.setPixelColor(i,10,255,100);                   
  }
    }
    strip.show();

    // Serial.println("Alliance?: " + alliance +       //Prints out the data above to the serial console
    //   ", Climbing?: "  + climbing + ", forwardDrive?: " + 
    //   forwardDrive + ", reverseDrive?: " + reverseDrive + ", still?:" +still+", visionLock?:"+visionLock+", xFinal?:"+xFinal); 
  
  }

  //Below here is code to control the LED's from the data obtained above
  // void reset(){
  // for(int i = 0; i < numberOfPixels; i++){       //Resets the full LED strip...
  //   strip.setPixelColor(i,0,0,0);                   //...by setting each LED to black
  // }
  // }
//  void reset(){
//  strip.begin();
//  strip.show(); //initialize all pixels to "off"
//  }
 if(allianceData=="1A"){                        //If the alliance is blue, set the base LED color to blue
     for(int i = 0; i < numberOfPixels; i++){
      strip.setPixelColor(i,0,0,255);
     }
} else if(allianceData=="2A"){                        //If the alliance is red, set the base LED color to red
    strip.setPixelColor(16,255,0,0);
} else if(allianceData== "3A"){                        //If no alliance is specified, set the base LED color to purple
    strip.setPixelColor(17,0,255,0);
}
  strip.show();
 

  if(stateData=="4R"){
    for(int i = 0; i < numberOfPixels; i++){
      strip.setPixelColor(i,66,229,244);
      strip.show();
    }
  } else if(stateData=="2R"){
    for(int i = 0; i < numberOfPixels; i++){
      strip.setPixelColor(i,r,b,0);
      strip.show();
    }
   
  } else if(stateData=="0R"){
    for(int i = 0; i < numberOfPixels; i++){
      strip.setPixelColor(i,200,200,0);
      strip.show();
    }
    
  } else if(stateData=="1R") {
      for(int i = 0; i < numberOfPixels; i++){
        strip.setPixelColor(i,200,200,200);
        strip.show();
    }
        
  } else if(stateData=="3R"){
    for(int i = 0; i < numberOfPixels; i++){
       strip.setPixelColor(i,200,200,200);
       strip.show();
    }
    
  }
  
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

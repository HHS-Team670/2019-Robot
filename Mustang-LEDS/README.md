# Mustang LEDs
Code for using an Arduino to interface with the RoboRIO over and Ethernet shield and receive commands for color changing LEDs

## Additional Information:

### Parts List

* RoboRIO
* Arduino with Ethernet. This can be an Arduino with an Ethernet shield, or something like the Yun which has it built in
* Neopixel strip (WS2812 Integrated Light Source)
* Ethernet switch for extra Ethernet ports, so you can use Ethernet on RoboRio both for all your necessities and the fancy lights too

### Setup (Wiring + Code)

Neopixels connected to Arduino pin 6. Using GRB format at 800KHZ bitstream

![LEDS to Arduino](https://lh3.googleusercontent.com/-gFxHv0gW18o/XDo4LhdlO5I/AAAAAAABkuw/7t_E-fPhBMYgM4TPLr6VMoqpOA2vIxaYQCK8BGAs/s512/8225756110042778619%253Faccount_id%253D0)

* Pin 6 on Arduino/Ethernet shield to white wire (DATAIN) on Neopixel strip 
* 5v pin on Arduino/Ethernet shield to red wire (VIN) on Neopixel strip
* GND pin on Arduino/Ethernet shield to black wire (GND) on Neopixel strip
* Arduino Ethernet shield to robot radio (as long as it's somehow connected to Ethernet on the robot side)
* Arduino to USB power 


In the Arduino code, to set up IP addresses: 

```cpp

IPAddress ip(10,te,am,3);                            //Defines a static IP for the Arduino/Ethernet Shield.
                                                     // For example: 10,6,70 for ours
IPAddress robotIp(10,te,am,2);                       //Defines the robot's IP

...
    
robotClient.connect(robotIp, 5801);               //Connects the client instance to the robot's socket at 5801;
```

On the RIO-side, make sure you have this in robotInit:

```java
 public void robotInit() {
    ...
    leds.socketSetup(5801);
    ...
  }
```

To run on RIO, deploy robotside code with gradle `./gradlew deploy` and also upload the Arduino sketch to the Arduino.

## Branching and Workflow on this Repository<br/>
Please check this document for the team's policy for committing code to GitHub!<br/>
https://docs.google.com/document/d/1vO_dtVTDw3-l0x0BabiAE5C45O6bJlQeLL1Uy9McOcQ/edit?usp=sharing <br/>
**Note that you cannot commit directly to master or dev!**<br/>
This project shall follow the following workflow:<br/>

The master branch is considered the stable branch of this project. It may only be updated via pull request from student developer, and then only with Code Leads' approval.<br/>

The dev branch is the main working branch. It may only be updated by pull request from uncontrolled branches.<br/>

For regular development each developer shall create a "feature branch" this is a branch named in the convention: "feature/name" or "bugfix/name". These are for new features and for bugfixes, respectively.<br/>

When work starts on a new feature, its branch will be made off of the latest version of dev, and all development will occur on the branch. When the feature is considered ready, it will be merged onto the dev branch. When merging, automatic merging, LV Merge tool merging, or simply copying and pasting of code fragments may be necessary.

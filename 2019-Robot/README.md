# RoboRIO-Side Code for the Robot

Uses GradleRIO to build and deploy code.

## Directory Hierarchy ##

</b>/main</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;```/deploy```: Files to be deployed to the RoboRIO (ex. Pathfinder .traj or .csv paths)</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;```/java/frc/team670```: Any code written by Team 670</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;```/commands```: All Commands and CommandGroups</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;```/constants```: All global constants that need to be stored</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;```/subsystem```: All Subsystems</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;```/utils```: All utility classes (calculations or things on the robot that are not Subsystems)</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;```/dataCollection```: All data collected from sesnors, vision on raspberry pi, etc.</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; ```Robot.java```: The "main" class on our end for when code is running on the RoboRIO</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp; ```Main.java```: A class to run test stuff not dependent on running off the RoboRIO</b>
<br><br>
&nbsp;&nbsp;&nbsp;&nbsp;```/java/frc/teamXXX```: Any code written not by Team 670</b>

## Use ##

Build with ```./gradlew build``` </b> <br><br>
Deploy while connected to the robot with ```./gradlew deploy``` You may need to build while connected to WiFi before being able to deploy </b> <br><br>
Launch the SmartDashboard with ```./gradlew smartdashboard```

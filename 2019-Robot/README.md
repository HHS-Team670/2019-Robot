# RoboRIO-Side Code for the Robot

Uses GradleRIO to build and deploy code.

Directory Hierarchy goes as follows:</b>
/main
.../deploy: Files to be deployed to the RoboRIO (ex. Pathfinder .traj or .csv paths)
.../java/frc/team670: Any code written by Team 670
... ... /commands: All Commands and CommandGroups
... ... /constants: All global constants that need to be stored
... ... /subsystems: All Subsystems
... ... /utils: All utility classes (calculations or things on the robot that are not Subsystems)
... ... ... /dataCollection: All data collected from sesnors, vision on raspberry pi, etc.
... ... Robot.java: The "main" class on our end for when code is running on the RoboRIO
... Main.java: A class to run test stuff not dependent on running off the RoboRIO
.../java/frc/teamXXX: Any code written not by Team 670

Build with './gradlew build' </b>
Deploy while connected to the robot with './gradlew deploy' You may need to build while connected to WiFi before being able to deploy </b>
Launch the SmartDashboard with './gradlew smartdashboard'
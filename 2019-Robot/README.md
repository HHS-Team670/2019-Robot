# RoboRIO-Side Code for the Robot

Uses GradleRIO to build and deploy code.

Directory Hierarchy goes as follows:</b>
/main</b>
.../deploy: Files to be deployed to the RoboRIO (ex. Pathfinder .traj or .csv paths)</b>
.../java/frc/team670: Any code written by Team 670</b>
... ... /commands: All Commands and CommandGroups</b>
... ... /constants: All global constants that need to be stored</b>
... ... /subsystems: All Subsystems</b>
... ... /utils: All utility classes (calculations or things on the robot that are not Subsystems)</b>
... ... ... /dataCollection: All data collected from sesnors, vision on raspberry pi, etc.</b>
... ... Robot.java: The "main" class on our end for when code is running on the RoboRIO</b>
... Main.java: A class to run test stuff not dependent on running off the RoboRIO</b>
.../java/frc/teamXXX: Any code written not by Team 670</b>

Build with './gradlew build' </b>
Deploy while connected to the robot with './gradlew deploy' You may need to build while connected to WiFi before being able to deploy </b>
Launch the SmartDashboard with './gradlew smartdashboard'

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

public class Pose {

  private long leftEncoderTick, rightEncoderTick; 
  private double robotAngle;

  private long currRobotX, currRobotY;
  

  public Pose(long lEncoderTick, long rEncoderTick, double angle) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    robotAngle = angle;
  }
 

  //Updates the robot's position and angle every time updatePose is called
  public void updatePose(long lEncoderTick, long rEncoderTick, double angle){
    
    long lDeltaTick = lEncoderTick - leftEncoderTick;
    long rDeltaTick = rEncoderTick - rightEncoderTick;
    long hypotenuse = (lDeltaTick+rDeltaTick)/2;
    
    double deltaAngle = (angle - robotAngle)/2;

    currRobotX = (long) (Math.cos(deltaAngle*(Math.PI/180)) * hypotenuse);
    currRobotY = (long) (Math.sin(deltaAngle*(Math.PI/180)) * hypotenuse);

    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    robotAngle = angle;
  }

  public long getPosX(){
    return currRobotX;
  }

  public long getPosY(){
    return currRobotY;
  }

 public double getLeftEncoderTick(){
    return leftEncoderTick;
  }

  public double getRightEncoderTick(){
    return rightEncoderTick;
  }
  public double getRobotAngle(){
    return robotAngle;
  }
}

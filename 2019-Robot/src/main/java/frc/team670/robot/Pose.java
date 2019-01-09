/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

public class Pose {

  private static double leftEncoderTick, rightEncoderTick; 
  private static double robotAngle;
  

  public Pose(double leftEncoderTick, double rightEncoderTick, double robotAngle) {
    
    setLeftEncoderTick(leftEncoderTick);
    setRightEncoderTick(rightEncoderTick);
    setRobotAngle(robotAngle);
    
  }

  private double getLeftEncoderTick(){
    return leftEncoderTick;
  }

  private double getRightEncoderTick(){
    return rightEncoderTick;
  }
  private double getRobotAngle(){
    return robotAngle;
  }



  private void setLeftEncoderTick(double leftEncoderTick) {
    this.leftEncoderTick = leftEncoderTick;
  }

  private void setRightEncoderTick(double rightEncoderTick) 
  {
    this.rightEncoderTick = rightEncoderTick;

  }

  private void setRobotAngle(double robotAngle) {
    this.robotAngle = robotAngle;
  }

  

  
}

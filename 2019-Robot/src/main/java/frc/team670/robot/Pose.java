/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import frc.team670.robot.utils.functions.MathUtils;

public class Pose {

  private static double leftEncoderTick, rightEncoderTick; 
  private static double robotAngle;
  

  public Pose(double leftEncoderTick, double rightEncoderTick, double robotAngle) {
    
    setLeftEncoderTick(leftEncoderTick);
    setRightEncoderTick(rightEncoderTick);
    setRobotAngle(robotAngle);
    
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



  public void setLeftEncoderTick(double leftEncoderTick) {
    this.leftEncoderTick = leftEncoderTick;
  }

  public void setRightEncoderTick(double rightEncoderTick) 
  {
    this.rightEncoderTick = rightEncoderTick;

  }

  public void setRobotAngle(double robotAngle) {
    this.robotAngle = robotAngle;
  }

  public double averageOfEncoders() {
    return MathUtils.average(leftEncoderTick, rightEncoderTick);
  }
  

  
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import frc.team670.robot.Robot;

public class Pose {

  private long leftEncoderTick, rightEncoderTick; 
  private double currentAngle;

  private long currRobotX, currRobotY;

  private long timeOfPose;
  
  /**
   * Makes new pose using current robot encoder values and angles
   */
  public Pose() {
    this(Robot.driveBase.getLeftEncoder(), Robot.driveBase.getRightEncoder(), Robot.sensors.getYawDouble());
  }

  /**
   * 
   * @param lEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param rEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param angle Angle in degrees -> left = negative, right = positive
   */
  public Pose(long lEncoderTick, long rEncoderTick, double angle) {
    this(lEncoderTick, rEncoderTick, angle, 0, 0, System.currentTimeMillis());
  }

  public Pose(double x, double y, double angle) {
    currRobotX = (long)x;
    currRobotY = (long)y;
    currentAngle = angle;
    timeOfPose = System.currentTimeMillis();
  }

  private Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY, long timeOfPose) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
    this.timeOfPose = timeOfPose;
  }

  private Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
  }
 
  /**
   * Updates the Pose's position and angle corresponding to the drivebase's ticks and NavX gyro reading.
   */
  public void update(long newLeftEncoderTick, long newRightEncoderTick, double newAngle){
    
    long lDeltaTick = newLeftEncoderTick - leftEncoderTick;
    long rDeltaTick = newRightEncoderTick - rightEncoderTick;
    long hypotenuse = (lDeltaTick+rDeltaTick)/2;
    
    double deltaAngle = (newAngle - currentAngle)/2;

    currRobotX = (long) (Math.cos(deltaAngle*(Math.PI/180)) * hypotenuse);
    currRobotY = (long) (Math.sin(deltaAngle*(Math.PI/180)) * hypotenuse);

    leftEncoderTick = newLeftEncoderTick;
    rightEncoderTick = newRightEncoderTick;
    currentAngle = newAngle;
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
    return currentAngle;
  }

  /**
   * Resets the Robot's NavX angle and the drive base encoders to 0, essentially changing the Pose inputs to zero.
   * Use this before you create a new Pose for a driving Command that utilizes it to ensure accuracy.
   */
  public static void resetPoseInputs() {
    Robot.sensors.resetNavX();
    Robot.driveBase.resetEncoders();
  }

  /**
   * @return the timeOfPose
   */
  public long getTimeOfPose() {
    return timeOfPose;
  }

  public Pose clone() {
    return new Pose(leftEncoderTick, rightEncoderTick, currentAngle, currRobotX, currRobotY, timeOfPose);
  }
}

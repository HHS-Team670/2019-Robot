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
  private double leftVelocity, rightVelocity;
  private double currentAngle;
  private double averagedAngle; // for testing purposes

  private long currRobotX, currRobotY;

  private long timeOfPose;

  /**
   * Makes new pose using current robot encoder values and angles. Does everything in ticks 
   */
  public Pose() {
   this(Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks(), Robot.sensors.getYawDouble(), Robot.driveBase.getLeftMustangEncoderVelocityInTicksPerSecond(), Robot.driveBase.getRightMustangEncoderVelocityInTicksPerSecond());  
  }

  /**
   * 
   * @param lEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param rEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param angle        Angle in degrees -> left = negative, right = positive
   */
  public Pose(long lEncoderTick, long rEncoderTick, double angle, double leftVelocity, double rightVelocity) {
    this(lEncoderTick, rEncoderTick, angle, 0, 0, System.currentTimeMillis(), leftVelocity, rightVelocity, 0);
  }

  public Pose(double x, double y, double angle, int leftEncoderPosition, int rightEncoderPosition, int leftVelocity, int rightVelocity) {
    this(leftEncoderPosition, rightEncoderPosition, angle, leftVelocity, rightVelocity);
    currRobotX = (long) x;
    currRobotY = (long) y;
  }

public Pose(double x, double y, double angle, int leftEncoderPosition, int rightEncoderPosition, double leftVelocity, double rightVelocity) {
    currRobotX = (long)x;
    currRobotY = (long)y;
    currentAngle = angle;
    leftEncoderTick = leftEncoderPosition;
    rightEncoderTick = rightEncoderPosition;
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    timeOfPose = System.currentTimeMillis();
  }

  private Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY, long timeOfPose, double leftVelocity, double rightVelocity, double averagedAngle) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    this.timeOfPose = timeOfPose;
    this.averagedAngle = averagedAngle;
  }

  public Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY, int leftVelocity,
      int rightVelocity) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
    this.timeOfPose = System.currentTimeMillis();
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
  }

  /**
   * Updates the Pose's position and angle corresponding to the drivebase's ticks
   * and NavX gyro reading.
   */
  public void update(long newLeftEncoderTick, long newRightEncoderTick, double newAngle, double leftVelocity, double rightVelocity){
    
    long lDeltaTick = newLeftEncoderTick - leftEncoderTick;
    long rDeltaTick = newRightEncoderTick - rightEncoderTick;
    long hypotenuse = (lDeltaTick + rDeltaTick) / 2;

    if ((newAngle > 90 && currentAngle < -90) || (newAngle < -90 && currentAngle > 90)) {
      if (newAngle < 0) {
        newAngle += 360;
      } else if (currentAngle < 0) {
        currentAngle += 360;
      }
    }

    averagedAngle = (newAngle + currentAngle) / 2;

    while (averagedAngle > 180) {
      averagedAngle -= 360;
    }
    while (averagedAngle < -180) {
      averagedAngle += 360;
    }
    
    currRobotX += (long) (Math.cos(averagedAngle * (Math.PI / 180)) * hypotenuse);
    currRobotY += (long) (Math.sin(averagedAngle * (Math.PI / 180)) * hypotenuse);

    leftEncoderTick = newLeftEncoderTick;
    rightEncoderTick = newRightEncoderTick;
    currentAngle = newAngle;

    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    timeOfPose = System.currentTimeMillis();
  }

  public void update() {
    update(Robot.driveBase.getLeftMustangEncoderPositionInTicks(), Robot.driveBase.getRightMustangEncoderPositionInTicks(), Robot.sensors.getYawDouble(),
           Robot.driveBase.getLeftMustangEncoderVelocityInTicksPerSecond(), Robot.driveBase.getRightMustangEncoderVelocityInTicksPerSecond());
  }

  /**
   * The X Position in Mustang encoder ticks
   */
  public long getPosX(){
    return currRobotX;
  }

  /**
   * The Y Position in Mustang Encoder ticks
   */
  public long getPosY(){
    return currRobotY;
  }

  /**
   * The left tick value in Mustang Encoder ticks
   */
 public double getLeftEncoderTick(){
    return leftEncoderTick;
  }

  /**
   * The right tick value in Mustang Encoder ticks
   */
  public double getRightEncoderTick(){
    return rightEncoderTick;
  }

  public double getRobotAngle() {
    return currentAngle;
  }

  /**
   * @return the timeOfPose
   */
  public long getTimeOfPose() {
    return timeOfPose;
  }

  /**
   * Gets the left side of the robot's velocity at the time of the pose in ticks/second using Mustang Encoder ticks
   */
  public double getLeftVelocity() {
    return leftVelocity;
  }

  /**
   * Gets the right side of the robot's velocity at the time of the pose in ticks/second using Mustang Encoder ticks
   */
  public double getRightVelocity() {
    return rightVelocity;
  }

  // For testing purposes
  public double averagedAngle() {
    return averagedAngle;
  }

  /**
   * Returns a copy of this Pose
   */
  public Pose clone() {
    return new Pose(leftEncoderTick, rightEncoderTick, currentAngle, currRobotX, currRobotY, timeOfPose, leftVelocity,
        rightVelocity, averagedAngle);
  }

}

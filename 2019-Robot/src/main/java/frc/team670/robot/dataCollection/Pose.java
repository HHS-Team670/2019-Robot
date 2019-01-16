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
  private int leftVelocity, rightVelocity;
  private double currentAngle;

  private long currRobotX, currRobotY;

  private long timeOfPose;
  
  private static Pose fieldCentricPose;

  /**
   * Makes new pose using current robot encoder values and angles
   */
  public Pose() {
    this(Robot.driveBase.getLeftEncoderPosition(), Robot.driveBase.getRightEncoderPosition(), Robot.sensors.getYawDouble());
  }

  /**
   * 
   * @param lEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param rEncoderTick Encoder ticks -> forward = positive, backwards = negative
   * @param angle Angle in degrees -> left = negative, right = positive
   */
  public Pose(long lEncoderTick, long rEncoderTick, double angle) {
    this(lEncoderTick, rEncoderTick, angle, 0, 0, System.currentTimeMillis(), Robot.driveBase.getLeftVelocity(), Robot.driveBase.getRightVelocity());
  }

  public Pose(double x, double y, double angle) {
    currRobotX = (long)x;
    currRobotY = (long)y;
    currentAngle = angle;
    leftEncoderTick = Robot.driveBase.getLeftEncoderPosition();
    rightEncoderTick = Robot.driveBase.getRightEncoderPosition();
    leftVelocity = Robot.driveBase.getLeftVelocity();
    rightVelocity = Robot.driveBase.getRightVelocity();
    timeOfPose = System.currentTimeMillis();
  }

  private Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY, long timeOfPose, int leftVelocity, int rightVelocity) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    this.timeOfPose = timeOfPose;
  }

  public Pose(long lEncoderTick, long rEncoderTick, double angle, long currRobotX, long currRobotY) {
    leftEncoderTick = lEncoderTick;
    rightEncoderTick = rEncoderTick;
    currentAngle = angle;
    this.currRobotX = currRobotX;
    this.currRobotY = currRobotY;
    this.timeOfPose = System.currentTimeMillis();
    leftVelocity = Robot.driveBase.getLeftVelocity();
    rightVelocity = Robot.driveBase.getRightVelocity();
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

    leftVelocity = Robot.driveBase.getLeftVelocity();
    rightVelocity = Robot.driveBase.getRightVelocity();
    timeOfPose = System.currentTimeMillis();
  }

  public void update() {
    update(Robot.driveBase.getLeftEncoderPosition(), Robot.driveBase.getRightEncoderPosition(), Robot.sensors.getYawDouble());
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
   * @return the timeOfPose
   */
  public long getTimeOfPose() {
    return timeOfPose;
  }

  /**
   * Gets the left side of the robot's velocity at the time of the pose in ticks/second
   */
  public int getLeftVelocity() {
    return leftVelocity;
  }

  /**
   * Gets the right side of the robot's velocity at the time of the pose in ticks/second
   */
  public int getRightVelocity() {
    return rightVelocity;
  }

  public Pose clone() {
    return new Pose(leftEncoderTick, rightEncoderTick, currentAngle, currRobotX, currRobotY, timeOfPose, leftVelocity, rightVelocity);
  }

  /**
   * Instantiates the field centric Pose. Call this in Robot.autonomousInit()
   */
  public static void instantiateFieldCentricPose () {
    fieldCentricPose = new Pose();
  }

  /**
   * Updates the field centric Pose that is stored
   */
  public static void updateFieldCentricPose() {
    fieldCentricPose.update();
  }

  /**
   * Gets a Pose that is kept up to date with the robot field centrically.
   */
  public static Pose getFieldCentricPose() {
    return fieldCentricPose;
  }
}

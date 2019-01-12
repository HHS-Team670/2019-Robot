/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;

/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {

  // TODO declare all of the correct motor controllers. Add them to ArrayLists for easily changing parameters for all of them. 
  // Make helper methods to change them and access sensor data. Set a DefaultCommand

  private CANSparkMax left1, left2, right1, right2;
  private SpeedControllerGroup left, right;
  private DifferentialDrive driveTrain;

  public DriveBase() {
    driveTrain = new DifferentialDrive(left, right);
     }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed Speed for left side of drive base [-1, 1]
   * @param rightSpeed Speed for right side of drive base [-1, 1]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  public void initAutonDrive(){
    //TODO -- make this set up the auton drive's settings
  }

  /**
   * 
   * Drives the Robot using a tank drive configuration (two joysticks, or auton)
   * 
   * @param leftSpeed Speed for left side of drive base [-1, 1]
   * @param rightSpeed Speed for right side of drive base [-1, 1]
   * @param squaredInputs If true, decreases sensitivity at lower inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) { 
    driveTrain.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  /**
   * 
   * Drives the Robot using a curvature drive configuration (wheel)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   * @param isQuickTurn If true enables turning in place and running one side backwards to turn faster
   */
  public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
    driveTrain.curvatureDrive(xSpeed, zRotation, isQuickTurn);
  }

   /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with twist)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   * @param isQuickTurn If true, decreases sensitivity at lower inputs
   */
  public void arcadeDrive(double xSpeed, double zRotation, boolean squaredInputs) {
    driveTrain.arcadeDrive(xSpeed, zRotation, squaredInputs);
  }

  public int getLeftEncoderPosition(){
    return 0; // TODO make this work
  }

  /**
   * Gets the position of the front right motor, this encoder gets more positive as it goes forward
   */
  public int getRightEncoderPosition(){
    return 0; // TODO make this work.
  }



  /**
   * 
   * Drives the Robot using an arcade drive configuration (single joystick with twist)
   * 
   * @param xSpeed The forward throttle speed [-1, 1]
   * @param zRotation The amount of rotation to turn [-1, 1] with positive being right
   */
  public void arcadeDrive(double xSpeed, double zRotation) {
    arcadeDrive(xSpeed, zRotation, false);
  }

  public void stop() {
    tankDrive(0, 0);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

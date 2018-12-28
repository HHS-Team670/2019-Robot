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

/**
 * Represents a tank drive base.
 * 
 * @author shaylandias
 */
public class DriveBase extends Subsystem {

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


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}

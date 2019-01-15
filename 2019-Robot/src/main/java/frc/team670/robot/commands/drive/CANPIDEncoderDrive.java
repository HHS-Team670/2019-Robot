/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import com.revrobotics.CANPIDController;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.DriveBase;

public class CANPIDEncoderDrive extends Command {

  private double finalDistance;
  private int leftEncoderPosition, rightEncoderPosition;
  private double threshold = 300;//TODO Define threshold
  private DriveBase robotDriveBase = new DriveBase();
  private CANPIDController LeftCANPIDEncoderController, RightCANPIDEncoderController;

  public CANPIDEncoderDrive(double finalDistance) {
    LeftCANPIDEncoderController = new CANPIDController(robotDriveBase.getLeftMotor());
    RightCANPIDEncoderController = new CANPIDController(robotDriveBase.getRightMotor());
    leftEncoderPosition = robotDriveBase.getLeftEncoderPosition();
    rightEncoderPosition = robotDriveBase.getRightEncoderPosition();
    this.finalDistance = finalDistance;

    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;

public class TestVelocityDrive extends CommandBase {
  private double leftVelocity, rightVelocity;

  public TestVelocityDrive(double leftVelocity, double rightVelocity) {
    addRequirements(Robot.driveBase);
    this.leftVelocity = leftVelocity;
    this.rightVelocity = rightVelocity;
    withTimeout(2);
  }

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    Robot.driveBase.tankDrive(leftVelocity/40, leftVelocity/40);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    //TODO find command that replaces isTimedOut() in 2020 wpilib
    return isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    Robot.driveBase.stop();

  }

  /*
  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  public void interrupted() {
    end();
  }
  */
}
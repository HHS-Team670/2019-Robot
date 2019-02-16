/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.RotatingSubsystem;
import frc.team670.robot.utils.functions.MathUtils;

public class MoveRotatorToSetpoint extends Command {

  private RotatingSubsystem rotatingSubsystem;
  private double setpointAngle;

  public MoveRotatorToSetpoint(RotatingSubsystem rotatingSubsystem, double setpointAngle) {
    requires(rotatingSubsystem);
    this.rotatingSubsystem = rotatingSubsystem;
    this.setpointAngle = setpointAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rotatingSubsystem.setMotionMagicSetpointAngle(setpointAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //Need to change this if you want to stop
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rotatingSubsystem.stop();
    rotatingSubsystem.clearSetpoint();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}


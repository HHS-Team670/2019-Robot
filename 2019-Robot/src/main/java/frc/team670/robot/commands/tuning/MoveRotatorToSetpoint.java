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
  private int ticks;

  public MoveRotatorToSetpoint(RotatingSubsystem rotatingSubsystem, int ticks) {
    requires(rotatingSubsystem);
    this.rotatingSubsystem = rotatingSubsystem;
    this.ticks = ticks;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rotatingSubsystem.setMotionMagicSetpointTicks(ticks);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return MathUtils.isWithinTolerance(ticks, rotatingSubsystem.getPositionTicks(), 8);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    rotatingSubsystem.enablePercentOutput();
    rotatingSubsystem.clearSetpoint();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

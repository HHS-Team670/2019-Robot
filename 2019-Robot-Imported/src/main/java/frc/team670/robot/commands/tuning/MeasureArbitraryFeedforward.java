/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.subsystems.RotatingSubsystem;
import frc.team670.robot.subsystems.TunableSubsystem;

public class MeasureArbitraryFeedforward extends Command {

  private TunableSubsystem tunableSubsystem;
  public static double output;

  public MeasureArbitraryFeedforward(TunableSubsystem tunableSubsystem) {
    this.tunableSubsystem = tunableSubsystem;
    // if (tunableSubsystem.getTimeout())
    //   setTimeout(0.35);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    tunableSubsystem.stop();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    tunableSubsystem.moveByPercentOutput(output); 
    SmartDashboard.putNumber("Intake Movement Output", output);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // if (tunableSubsystem.getTimeout())
    //   return isTimedOut();
    // else 
      return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    tunableSubsystem.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}

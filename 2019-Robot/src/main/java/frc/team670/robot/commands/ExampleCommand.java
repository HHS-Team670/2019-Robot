/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.utils.Logger;

/**
 * An example command.  You can replace me with your own command.
 */
public class ExampleCommand extends Command {
  public ExampleCommand() {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog("Initialized Example");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    int example = 1;
    Logger.consoleLog("Executing Example: %s", example);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("Finished Example");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted Example");
    end();
  }
}

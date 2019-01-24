/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.Robot;
import frc.team670.robot.utils.functions.SettingUtils;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.constants.RobotConstants;

public class RetractBackPistons extends Command {
  private int loggingIterationCounter;

  public RetractBackPistons() {
    requires(Robot.climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climber.setBackPIDControllerSetpoint(RobotConstants.PISTON_ENCODER_FLAT);

    Logger.consoleLog("startBackPistonPosition:%s", Robot.climber.getBackTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("CurrentBackPistonPosition:%s", Robot.climber.getBackTalonPositionInTicks());

      loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climber.getBackController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SettingUtils.releaseController(Robot.climber.getBackController());
    Robot.climber.setBackPistonsRetracted(true);
    Logger.consoleLog("EndBackPistonPosition:%s", Robot.climber.getBackTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog("RetractBackPiston interrupted");
  }
}

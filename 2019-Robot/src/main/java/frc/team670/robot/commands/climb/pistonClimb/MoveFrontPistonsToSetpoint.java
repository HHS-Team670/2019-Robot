/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.Logger;


/**
 * Command to move the front pistons to a given setpoint
 */
public class MoveFrontPistonsToSetpoint extends Command {
  private int loggingIterationCounter;
  private int setpoint;
  private Climber climber;

  /**
   * 
   * @param setpoint The desired setpoint in ticks
   * @param climber The climber upon which this command will be used
   */
  public MoveFrontPistonsToSetpoint(int setpoint, Climber climber) {
    this.setpoint = setpoint;
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog("startFrontPistonPosition:%s", Robot.climber.getFrontTalonPositionInTicks());
    climber.setFrontPIDControllerSetpoint(setpoint);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Logger.consoleLog("CurrentFrontPistonPosition:%s", climber.getFrontTalonPositionInTicks());
    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getFrontControllerOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("EndFrontPistonPosition:%s", climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
  }
}

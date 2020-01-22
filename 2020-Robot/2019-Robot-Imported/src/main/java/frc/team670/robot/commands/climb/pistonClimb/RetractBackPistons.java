/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.Logger;

/**
 * Command to retract the back pistons all the way
 */
public class RetractBackPistons extends Command {
  private int loggingIterationCounter;
  private Climber climber;

  public RetractBackPistons(Climber climber) {
    requires(climber);
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.setBackPIDControllerSetpoint(Climber.PISTON_ENCODER_FLAT);
    climber.setBackPistonsRetractionInProgress(true);
    Logger.consoleLog("startBackPistonPosition:%s", climber.getBackTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Logger.consoleLog("CurrentBackPistonPosition:%s", climber.getBackTalonPositionInTicks());
    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getBackControllerOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.setBackPistonsRetracted(true);

    //Retraction is no longer in progress, so the pistons have been retracted all the way
    climber.setBackPistonsRetractionInProgress(false);
    Logger.consoleLog("BackPistonPosition:%s", climber.getBackTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog();
  }
}

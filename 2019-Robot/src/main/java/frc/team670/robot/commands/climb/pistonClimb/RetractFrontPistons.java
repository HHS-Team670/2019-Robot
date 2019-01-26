/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.utils.Logger;

/**
 * Command to retract the front pistons all the way
 */
public class RetractFrontPistons extends Command {
  private int loggingIterationCounter;
  private Climber climber;

  /**
   * 
   * @param climber The climber upon which this command will be called on
   */
  public RetractFrontPistons(Climber climber) {
    requires(Robot.climber);
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.setFrontPIDControllerSetpoint(RobotConstants.PISTON_ENCODER_FLAT);
    climber.setFrontPistonsRetractionInProgress(true);
    Logger.consoleLog("startFrontPistonPosition:%s", Robot.climber.getFrontTalonPositionInTicks());
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
    return climber.getFrontPistonsRetracted();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    climber.setFrontPistonsRetracted(true);

    //Retraction is no longer in progress, so the pistons have been retracted all the way
    climber.setFrontPistonsRetractionInProgress(false);
    Logger.consoleLog("EndFrontPistonPosition:%s", climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
  }
}

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
import frc.team670.robot.utils.functions.SettingUtils;

public class RetractBackPistons extends Command {
  private int loggingIterationCounter;
  private Climber climber;

  public RetractBackPistons(Climber climber) {
    this.climber = climber;
    requires(climber);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    climber.setBackPIDControllerSetpoint(RobotConstants.PISTON_ENCODER_FLAT);

    Logger.consoleLog("startBackPistonPosition:%s", climber.getBackTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("CurrentBackPistonPosition:%s", climber.getBackTalonPositionInTicks());

      loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getBackController().onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SettingUtils.releaseController(climber.getBackController());
    climber.setBackPistonsRetracted(true);
    Logger.consoleLog("EndBackPistonPosition:%s", climber.getBackTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog("RetractBackPiston interrupted");
  }
}

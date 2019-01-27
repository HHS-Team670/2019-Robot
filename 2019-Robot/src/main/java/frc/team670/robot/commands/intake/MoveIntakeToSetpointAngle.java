/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Command to move the intake to a certain angle
 * 
 */
public class MoveIntakeToSetpointAngle extends Command {

  private Intake intake;
  private int loggingIterationCounter, setpointInDegrees;
  private int toleranceInDegrees = 5;

  /**
   * @param setpoint angle in degrees that the intake is moving to
   * @param intake the intake upon which command will be called upon
   */
  public MoveIntakeToSetpointAngle(int setpointInDegrees, Intake intake) {
    requires(intake);
    this.intake = intake;
    this.setpointInDegrees = setpointInDegrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    intake.setMotionMagicSetpoint(setpointInDegrees);
    Logger.consoleLog("startIntakeAngle:%s", intake.getIntakeAngleInDegrees());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Logger.consoleLog("currentIntakeAngle:%s", intake.getIntakeAngleInDegrees());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (MathUtils.isWithinTolerance(intake.getIntakeAngleInDegrees(), intake.getMotionMagicSetpoint(), toleranceInDegrees));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("endIntakeAngle:%s", intake.getIntakeAngleInDegrees());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog();
  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Command to move the intake to a certain angle
 * 
 */
public class MoveIntakeToSetpointAngle extends Command {

  private BaseIntake intake;
  private int loggingIterationCounter, setpointInTicks;
  private static final int TOLERANCE_IN_TICKS = 10;

  /**
   * @param setpoint angle in degrees that the intake is moving to
   * @param intake the intake upon which command will be called upon
   */
  public MoveIntakeToSetpointAngle(int setpointInDegrees, BaseIntake intake) {
    requires(intake);
    this.intake = intake;
    this.setpointInTicks = Intake.convertIntakeDegreesToTicks(setpointInDegrees);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    intake.setRotatorNeutralMode(NeutralMode.Brake);
    intake.setMotionMagicSetpoint(setpointInTicks);
    Logger.consoleLog("startIntakeAngle:%s", intake.getIntakeAngleInDegrees());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Logger.consoleLog("currentIntakePosition:%s", intake.getIntakePositionInTicks());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (MathUtils.isWithinTolerance(intake.getIntakePositionInTicks(), setpointInTicks, TOLERANCE_IN_TICKS));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.setRotatorNeutralMode(NeutralMode.Coast);
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

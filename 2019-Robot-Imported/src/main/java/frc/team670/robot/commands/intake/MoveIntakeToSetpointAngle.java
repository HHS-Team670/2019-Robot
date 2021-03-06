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

/**
 * Command to move the intake to a certain angle
 * 
 */
public class MoveIntakeToSetpointAngle extends Command {

  private BaseIntake intake;
  private int loggingIterationCounter, setpointInDegrees;
  private static final int TOLERANCE_IN_DEGREES = 5;
  private static final double TIMEOUT = 1.75;

  /**
   * @param setpoint angle in degrees that the intake is moving to
   * @param intake the intake upon which command will be called upon
   */
  public MoveIntakeToSetpointAngle(int setpointInDegrees, BaseIntake intake) {
    requires(intake);
    setTimeout(TIMEOUT);
    this.intake = intake;
    this.setpointInDegrees = setpointInDegrees;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    intake.setRotatorNeutralMode(NeutralMode.Brake);
    intake.setMotionMagicSetpointAngle(setpointInDegrees);
    // Logger.consoleLog("startIntakeAngle:%s", intake.getAngleInDegrees());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Logger.consoleLog("currentIntakeAngle:%s", intake.getAngleInDegrees());
    if(setpointInDegrees < 0) {
      intake.runIntake(0.8, false);
    }

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isTimedOut()) {
      return true;
    }
    if(setpointInDegrees > 0) {
      return (intake.getAngleInDegrees() > setpointInDegrees - TOLERANCE_IN_DEGREES);
    } else {
      return (intake.getAngleInDegrees() < setpointInDegrees + TOLERANCE_IN_DEGREES);
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    intake.stop();
    intake.setRotatorNeutralMode(NeutralMode.Coast);
    intake.stopRollers();
    // Logger.consoleLog("endIntakeAngle:%s", intake.getAngleInDegrees());
    // System.out.println("FINISHED MOVE INTAKE");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog();
  }
}

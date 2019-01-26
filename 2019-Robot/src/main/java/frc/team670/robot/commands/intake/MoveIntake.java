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

public class MoveIntake extends Command {

  private Intake intake;
  private int loggingIterationCounter;
  private int toleranceInDegrees = 5;

  public MoveIntake(Intake intake) {
    requires(intake);
    this.intake = intake;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // If the intake is closer to the up postion, we assume that we want to move it
    // down
    if (Math.abs(intake.getBaseVictorPositionInTicks() - Intake.INTAKE_ANGLE_UP) < Math.abs(intake.getBaseVictorPositionInTicks() - Intake.INTAKE_ANGLE_DOWN)) {
      intake.setMotionMagicSetpoint(Intake.INTAKE_ANGLE_DOWN);
    } else {
      // If intake is instead closer to the down postion, we assume that we want to
      // move it up
      intake.setMotionMagicSetpoint(Intake.INTAKE_ANGLE_UP);
    }

    Logger.consoleLog("startIntakePosition:%s", intake.getBaseVictorPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("currentIntakePosition:%s", intake.getBaseVictorPositionInTicks());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Math.abs(intake.getIntakeAngleInDegrees() - intake.getMotionMagicSetpoint()) < toleranceInDegrees);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("endIntakePosition:%s", intake.getBaseVictorPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("MoveIntake interrupted");
  }
}

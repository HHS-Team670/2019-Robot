/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Intake;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Command to toggle the intake up or down
 */
public class ToggleIntakeUpDown extends Command {

  private Intake intake;
  private int loggingIterationCounter;
  private int intakeSetpointInTicks;

  private int toleranceInTicks = 10;

  public ToggleIntakeUpDown(Intake intake, Arm arm) {
    requires(intake);
    this.intake = intake;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      // If the intake is closer to the up postion, we assume that we want to move it
      // down
    if (Math.abs(intake.getIntakePositionInTicks() - Intake.INTAKE_ANGLE_IN) < Math.abs(intake.getIntakePositionInTicks() - Intake.INTAKE_ANGLE_DEPLOYED)) {
      intakeSetpointInTicks = Intake.convertIntakeDegreesToTicks(Intake.INTAKE_ANGLE_DEPLOYED);
    } else {
      // If intake is instead closer to the down postion, we assume that we want to
      // move it up
      intakeSetpointInTicks = Intake.convertIntakeDegreesToTicks(Intake.INTAKE_ANGLE_IN);
    }

    double intakeHighPoint = Intake.INTAKE_FIXED_LENGTH_IN_INCHES + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES;
    double lowestPointOnClaw = Arm.getCurrentState().getMaximumLowestPointOnClaw();

    //TODO: Check if arm is moving when this command is called.
    
    if(intakeHighPoint >= lowestPointOnClaw && (Arm.getCurrentState().getCoordPosition().getX() > 0)){
      super.cancel();
      return;
    }
    intake.setMotionMagicSetpoint(intakeSetpointInTicks);
    Logger.consoleLog("startIntakePosition:%s, intakeSetpointInTicks", intake.getIntakePositionInTicks(), intakeSetpointInTicks);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  
    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (MathUtils.isWithinTolerance(intake.getIntakePositionInTicks(), intake.getMotionMagicSetpoint(), toleranceInTicks));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("endIntakePosition:%s", intake.getIntakePositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
  }
}

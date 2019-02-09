/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.zero;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.elbow.Elbow;
import frc.team670.robot.utils.Logger;

/**
 * Zeroes the Elbow encoder by slowly driving it to its back limit switch and resetting its value.
 */
public class ElbowBackReset extends Command {
  private Elbow elbow;

  public ElbowBackReset(Elbow elbow) {
    requires(elbow);
    this.elbow = elbow;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      elbow.setOutput(-0.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return elbow.isReverseLmitPressed();
  }

  /**
   * Resets elbow position to known max distance back
   */
  @Override
  protected void end() {
    elbow.setQuadratureEncoder(Elbow.MAX_ELBOW_BACK);
    elbow.setOutput(0);
    Logger.consoleLog();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    elbow.setOutput(0);
    Logger.consoleLog();
  }
}

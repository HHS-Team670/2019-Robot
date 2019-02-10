/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.zero;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.Logger;

/**
 * Zeroes the Wrist encoder by slowly driving it to its front limit switch and resetting its value.
 */
public class WristForwardReset extends Command {
  private Wrist wrist;

  public WristForwardReset(Wrist wrist) {
    requires(wrist);
    this.wrist = wrist;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      wrist.setOutput(0.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return wrist.isReverseLimitPressed();
  }

  /**
   * Resets elbow position to known max distance back
   */
  @Override
  protected void end() {
    wrist.setQuadratureEncoder(Wrist.FORWARD_SOFT_LIMIT);
    wrist.setOutput(0);
    Logger.consoleLog();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    wrist.setOutput(0);
    Logger.consoleLog();
  }
}

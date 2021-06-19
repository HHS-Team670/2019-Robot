/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.zero;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.extension.Extension;
import frc.team670.robot.utils.Logger;

/**
 * Don't use this!!! Zeroes the Extension encoder by slowly driving it to its forward limit switch and resetting its value.
 * @deprecated
 */
public class ExtensionForwardReset extends Command {
  private Extension extension;

  public ExtensionForwardReset(Extension extension) {
    requires(extension);
    this.extension = extension;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      extension.setOutput(0.1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return extension.isForwardLimitPressed();
  }

  /**
   * Resets elbow position to known max distance back
   */
  @Override
  protected void end() {
    extension.setQuadratureEncoder(Extension.EXTENSION_OUT_POS);
    extension.setOutput(0);
    Logger.consoleLog();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    extension.setOutput(0);
    Logger.consoleLog();
  }
}

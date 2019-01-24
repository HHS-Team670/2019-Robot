/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.zero;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.Extension;

/**
 * Zeroes the Extension encoder by slowly driving it to its back limit switch and resetting its value.
 */
public class ExtensionBackReset extends Command {
  private Extension extension;

  public ExtensionBackReset(Extension extension) {
    requires(extension);
    this.extension = extension;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      extension.setOutput(-0.2);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return extension.getReverseLimitSwitch();
  }

  /**
   * Resets elbow position to known max distance back
   */
  @Override
  protected void end() {
    extension.resetExtension(Extension.MAX_EXTENSION_BACK);
    extension.setOutput(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    extension.setOutput(0);
  }
}

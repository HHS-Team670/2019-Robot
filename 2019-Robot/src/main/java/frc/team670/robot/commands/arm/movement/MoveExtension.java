/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.subsystems.extension.Extension;

/*
 * Moves the Extension to an absolute inch value using MotionMagic. Use this as part of an ArmTransition.
 */
public class MoveExtension extends Command {

  private static final double DISTANCE_TOLERANCE = 10;

  private BaseExtension extension;
  private double extensionSetpointInTicks;
  private long executeCount;
  

  /**
   * Instantiates Command
   * @param wrist The Wrist to move. This could be the actual wrist from Robot, or a TestWrist
   * @param distance The absolute distance to move to [0, farthest extension possible] moving outwards with 0 at no extension.
   */
  public MoveExtension(BaseExtension extension, double distance) {
    this.extension = extension;
    requires(extension);
    extensionSetpointInTicks = Extension.convertExtensionInchesToTicks(distance);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    extension.setMotionMagicSetpoint(extensionSetpointInTicks);
    executeCount = 0;
    Logger.consoleLog("extensionSetpointInTicks: %s", extensionSetpointInTicks);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return MathUtils.isWithinTolerance(extension.getLengthTicks(), extensionSetpointInTicks, DISTANCE_TOLERANCE);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("extensionSetpointInTicks: %s, endingPositionInTicks: %s", extensionSetpointInTicks, extension.getLengthTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
    end();
  }
}
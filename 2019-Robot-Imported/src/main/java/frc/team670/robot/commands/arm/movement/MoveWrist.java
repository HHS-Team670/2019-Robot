/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Moves the Wrist to an absolute angle using MotionMagic. Use this as part of an ArmTransition.
 */
public class MoveWrist extends Command {

  private BaseWrist wrist;

  private static final double DEGREE_TOLERANCE = 8;

  private double wristSetpointAngle;

  private long executeCount;

  /**
   * Instantiates Command
   * @param wrist The Wrist to move. This could be the actual wrist from Robot, or a TestWrist
   * @param angle The absolute angle to move to (180, -180) with 180 being towards the front of the robot (where the intake is).
   * In reality, this angle will not be in this full range because the wrist will have a limit to how much it can move.
   */
  public MoveWrist(BaseWrist wrist, double angle) {
    this.wrist = wrist;
    wristSetpointAngle = angle;
    requires(wrist);
    super.setInterruptible(true);
    setTimeout(2);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    wrist.setMotionMagicSetpointAngle(wristSetpointAngle);
    executeCount = 0;
    // Logger.consoleLog("angleSetpoint: %s", wristSetpointAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isTimedOut()) {
      return true;
    }
    return MathUtils.isWithinTolerance(wrist.getAngleInDegrees(), wristSetpointAngle, DEGREE_TOLERANCE);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // Logger.consoleLog("targetAngleValue: %s, endingAngleValue: %s", wristSetpointAngle, wrist.getAngleInDegrees());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    wrist.setMotionMagicSetpointAngle(wrist.getAngleInDegrees());
    Logger.consoleLog();
    end();
  }
}

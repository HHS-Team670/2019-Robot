/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Moves the Wrist to an absolute angle using MotionMagic. Use this as part of an ArmTransition.
 */
public class MoveWrist extends Command {

  private BaseWrist wrist;
  private double angle;

  private static final double DEGREE_TOLERANCE = 0.5;

  private long executeCount;

  /**
   * Instantiates Command
   * @param wrist The Wrist to move. This could be the actual wrist from Robot, or a TestWrist
   * @param angle The absolute angle to move to (180, -180) with 180 being towards the front of the robot (where the intake is).
   * In reality, this angle will not be in this full range because the wrist will have a limit to how much it can move.
   */
  public MoveWrist(BaseWrist wrist, double angle) {
    this.wrist = wrist;
    requires(wrist);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    wrist.initializeMotionmagic();
    wrist.setMotionMagicSetpoint(angle);
    executeCount = 0;
    Logger.consoleLog("angle: %s", angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(executeCount % 5 == 0) {
      Logger.consoleLog("angle: %s", angle);
    }
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return MathUtils.isWithinTolerance(wrist.getAngle(), angle, DEGREE_TOLERANCE);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("targetAngle: %s, endingAngle: %s", angle, wrist.getAngle());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog();
    end();
  }
}

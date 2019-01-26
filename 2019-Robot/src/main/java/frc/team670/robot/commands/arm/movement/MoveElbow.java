/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Moves the Elbow to an absolute angle value using MotionMagic. Use this as part of an ArmTransition.
 */
public class MoveElbow extends Command {

  private BaseElbow elbow;
  private double angle;

  private static final double DEGREE_TOLERANCE = 0.5;

  private long executeCount;

  /**
   * Instantiates Command
   * @param elbow The Elbow to move. This could be the actual elbow from Robot, or a TestElbow
   * @param angle The absolute angle to move to (180, -180) with 180 being towards the front of the robot (where the intake is).
   * In reality, this angle will not be in this full range because the elbow will have a limit to how much it can move.
   */
  public MoveElbow(BaseElbow elbow, double angle) {
    this.elbow = elbow;
    requires(elbow);
    this.angle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    System.out.println("Initialized MoveElbow");
    elbow.initializeMotionmagic();
    executeCount = 0;
    Logger.consoleLog("MoveElbow: angle: %s", angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    System.out.println("Moved Elbow");
    elbow.setMotionMagicSetpoint(angle);
    if(executeCount % 5 == 0) {
      Logger.consoleLog("MoveElbow: angle: %s", angle);
    }
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return MathUtils.isWithinTolerance(elbow.getAngle(), angle, DEGREE_TOLERANCE);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("MoveElbow Ended");
    Logger.consoleLog("MoveElbow: targetAngle: %s, endingAngle: %s", angle, elbow.getAngle());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("MoveElbow Interrupted");
    end();
  }
}

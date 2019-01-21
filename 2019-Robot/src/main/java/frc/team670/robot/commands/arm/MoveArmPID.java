/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm.ArmState;
import edu.wpi.first.wpilibj.command.Command;

/**
 * PID controlling of arm as wrist, extension and elbow
 * TODO: once the states are figured out we can do more
 * @author ctchen, arleenliu
 */
public class MoveArmPID extends Command {

  private ArmState targetState;
  private final double ANGLE_TOLERANCE = 0.5, DISTANCE_TOLERANCE = 0.2;

  public MoveArmPID(ArmState state) {
    super();
    requires(Robot.extension);
    requires(Robot.wrist);
    requires(Robot.elbow);
    targetState = state;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.elbow.setMotionMagicSetpoint(targetState.getElbowAngle());
    Robot.extension.setMotionMagicSetpoint(targetState.getExtensionLength());
    Robot.wrist.setMotionMagicSetpoint(targetState.getWristAngle());
  }

  @Override
  protected void execute() {
    super.execute();
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  @Override
  protected void end() {
    super.end();
  }

  @Override
  protected void interrupted() {
    super.interrupted();
  }

}

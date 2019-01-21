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

  private ArmState currentState;
  private final double ANGLE_TOLERANCE, DISTANCE_TOLERANCE;
  private double 

  public MoveArmPID(ArmState state) {
    super();
    requires(Robot.extension);
    requires(Robot.wrist);
    requires(Robot.elbow);
    currentState = state;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.elbow.moveElbow(currentState.getElbowAngle());
    Robot.extension.moveExtension(currentState.getExtensionLength());
    Robot.wrist.moveWrist(currentState.getWristAngle());
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

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm.ArmState;

/**
 * PID controlling of arm as wrist, extension and elbow
 * TODO: once the states are figured out we can do more
 * @author ctchen, arleenliu
 */
public class MoveArmPID extends InstantCommand {
  private ArmState currentState;
  /**
   * Add your docs here.
   */
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

}

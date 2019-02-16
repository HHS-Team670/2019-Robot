/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Claw;

/**
 * Stops all non-drive base movement on the robot
 */
public class CancelArmMovement extends InstantCommand {
  private BaseIntake intake;

  /**
   * Add your docs here.
   */
  public CancelArmMovement(Arm arm, BaseIntake intake, Claw claw) {
    requires(arm.getWrist());
    requires(arm.getElbow());
    requires(arm.getExtension());
    requires(intake);
    requires(claw);

    this.intake = intake;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "CancelArmMovement");
    intake.runIntake(0, false);
  }

}

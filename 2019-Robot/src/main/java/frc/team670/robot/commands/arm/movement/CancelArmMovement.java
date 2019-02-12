/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.extension.BaseExtension;
import frc.team670.robot.subsystems.wrist.BaseWrist;

/**
 * Stops all non-drive base movement on the robot
 */
public class CancelArmMovement extends InstantCommand {
  private BaseIntake intake;

  /**
   * Add your docs here.
   */
  public CancelArmMovement(BaseElbow elbow, BaseExtension extension, BaseWrist wrist, BaseIntake intake, Claw claw) {
    requires(wrist);
    requires(elbow);
    requires(extension);
    requires(intake);
    requires(claw);

    this.intake = intake;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    intake.runIntake(0, false);
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.subsystems.BaseIntake;

/**
 * Stops the intake from runnign the rollers (sets speed to 0).
 */
public class StopIntakeRollers extends InstantCommand {

  private BaseIntake intake;

  public StopIntakeRollers(BaseIntake intake) {
    super();
    this.intake = intake;
    requires(intake);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "StopIntakeRollers");
    intake.runIntake(0, false);
  }

}

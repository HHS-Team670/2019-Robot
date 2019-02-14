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
 * Add your docs here.
 */
public class ButtonRunIntake extends InstantCommand {
  private BaseIntake intake;
  private boolean runningIn;
  private double power;

  public ButtonRunIntake(BaseIntake intake, double power, boolean runningIn) {
    this.intake = intake;
    this.runningIn = runningIn;
    this.power = power;
    SmartDashboard.putString("current-command", "ButtonRunIntake");

    intake.runIntake(power, runningIn);
  }
}

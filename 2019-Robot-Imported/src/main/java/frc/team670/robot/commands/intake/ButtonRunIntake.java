/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.dataCollection.XKeys;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class ButtonRunIntake extends InstantCommand {
  private BaseIntake intake;
  private boolean runningIn;
  private double power;

  public ButtonRunIntake(BaseIntake intake, double power, boolean runningIn) {
    requires(intake);
    this.intake = intake;
    this.runningIn = runningIn;
    this.power = power;
    XKeys.setToggleIn(runningIn);
  }


// Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "ButtonRunIntake");
    intake.runIntake(power, runningIn);
  }

}

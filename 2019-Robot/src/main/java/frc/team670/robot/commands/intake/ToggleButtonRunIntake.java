/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.intake;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.team670.robot.subsystems.BaseIntake;

public class ToggleButtonRunIntake extends ConditionalCommand {
  private boolean toggle;

  public ToggleButtonRunIntake(BaseIntake intake, boolean runningIn, boolean toggle) {
    super(new ButtonRunIntake(intake, RunIntakeInWithIR.RUNNING_POWER, runningIn), new ButtonRunIntake(intake, 0, runningIn));
    this.toggle = toggle;
  }

  @Override
  protected boolean condition() {
    return toggle;
  }
}

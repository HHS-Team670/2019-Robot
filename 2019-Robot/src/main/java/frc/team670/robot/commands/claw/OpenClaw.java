/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.Claw;

/**
 * Add your docs here.
 */
public class OpenClaw extends InstantCommand {
  
  private Claw claw;

  public OpenClaw(Claw claw) {
    super();
    this.claw = claw;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
  }

}

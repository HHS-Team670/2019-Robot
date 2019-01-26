/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import frc.team670.robot.subsystems.Claw;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Switches the claw between an opened/closed state with a hard/soft grip
 */
public class ToggleClawOpen extends InstantCommand {

  private Claw claw;
  private boolean soft;

  /**
   * @param claw The Claw object
   * @param soft True if the claw should use soft grip,
   */
  public ToggleClawOpen(Claw claw, boolean soft) {
    this.claw = claw;
    this.soft = soft;
    requires(claw);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

}

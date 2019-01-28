/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 * Toggles between opening and closing the Claw using soft open/close
 * @author shaylandias
 */
public class ToggleClaw extends InstantCommand {

  private Claw claw;

  /**
   * @param claw The Claw object
   */
  public ToggleClaw(Claw claw) {
    requires(claw);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    claw.toggleGrip();
    Logger.consoleLog();
  }

}
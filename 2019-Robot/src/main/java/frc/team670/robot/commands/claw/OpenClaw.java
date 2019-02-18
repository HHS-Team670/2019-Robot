/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 * Opens the Claw using the "soft" grip
 * @author shaylandias
 */
public class OpenClaw extends TimedCommand {

  private Claw claw;

  /**
   * @param claw The Claw object
   */
  public OpenClaw(Claw claw) {
    super(Claw.TIME_TO_MOVE);
    requires(claw);
    this.claw = claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(!claw.isOpen()) {
      claw.openClaw();
    }
    Logger.consoleLog();
  }

}

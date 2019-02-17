/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;

/**
 * Closes the Claw using the "soft" state.
 */
public class CloseClaw extends TimedCommand {
  
  private Claw claw;

  public CloseClaw(Claw claw) {
    super(Claw.TIME_TO_MOVE);
    this.claw = claw;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(claw.isOpen()) {
      claw.closeClaw(true);
    }
    Logger.consoleLog();
  }
}

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
 * Grabs the hatch by opening the intake hard.
 */
public class GrabHatch extends TimedCommand {
  
  private Claw claw;

  public GrabHatch(Claw claw) {
    super(Claw.TIME_TO_MOVE);
    this.claw = claw;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    claw.openClaw(false);
    Logger.consoleLog();
  }

  @Override
  protected boolean isFinished() {
    return super.isFinished();
  }

}

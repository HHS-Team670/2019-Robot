/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;

/**
 * Opens the Claw using the "soft" grip
 * @author shaylandias
 */
public class OpenClaw extends Command {

  private Claw claw;

  /**
   * @param claw The Claw object
   */
  public OpenClaw(Claw claw) {
    setTimeout(Claw.TIME_TO_MOVE);
    requires(claw);
    this.claw = claw;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    claw.openClaw();
    Robot.arm.setHeldItem(HeldItem.NONE);
    Logger.consoleLog();
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

}

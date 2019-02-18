/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;

/**
 * Grabs the hatch by opening the intake hard.
 */
public class GrabHatch extends TimedCommand {
  
  private Claw claw;
  private Arm arm;

  public GrabHatch(Claw claw, Arm arm) {
    super(Claw.TIME_TO_MOVE);
    this.claw = claw;
    this.arm = arm;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    claw.openClaw();
    Logger.consoleLog();
  }

  @Override
  protected void end() {
    super.end();
    arm.setHeldItem(HeldItem.HATCH);
  }

  @Override
  protected boolean isFinished() {
    return super.isFinished();
  }

  @Override
  protected void interrupted() {
    super.interrupted();
    end();
  }

}

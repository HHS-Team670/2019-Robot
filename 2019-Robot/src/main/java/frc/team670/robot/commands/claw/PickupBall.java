/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.wrist.Wrist;
import frc.team670.robot.subsystems.wrist.Wrist.HeldItem;
import frc.team670.robot.utils.Logger;

/**
 * Picks up the ball using soft close
 */
public class PickupBall extends TimedCommand {
 
  private Claw claw;
  private Wrist wrist;

  public PickupBall(Claw claw, Wrist wrist) {
    super(Claw.TIME_TO_MOVE);
    this.claw = claw;
    this.wrist = wrist;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    claw.closeClaw(true);
    Logger.consoleLog();
  }

  @Override
  protected boolean isFinished() {
    return super.isFinished();
  }

  @Override
  protected void end() {
    super.end();
    wrist.setHeldItem(HeldItem.BALL);
  }

  @Override
  protected void interrupted() {
    super.interrupted();
    end();
  }

}

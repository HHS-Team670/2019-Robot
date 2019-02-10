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
 * Drops the ball using a hard open and by pushing it
 */
public class DropBall extends TimedCommand {
  
  private Claw claw;

  public DropBall(Claw claw) {
    super(Claw.TIME_TO_MOVE);
    this.claw = claw;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    claw.openClaw(false);
    claw.push();
    Logger.consoleLog();
  }

  @Override
  protected void execute() {

  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
    super.end();
  }

  @Override
  protected void interrupted() {
    super.interrupted();
  }

}

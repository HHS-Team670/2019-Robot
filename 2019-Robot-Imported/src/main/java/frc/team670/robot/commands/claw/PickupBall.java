/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.utils.Logger;

/**
 * Picks up the ball using soft close
 */
public class PickupBall extends Command {
 
  private Claw claw;
  private Arm arm;

  public PickupBall(Claw claw, Arm arm) {
    setTimeout(Claw.TIME_TO_MOVE);
    this.claw = claw;
    this.arm = arm;
    requires(claw);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putString("current-command", "PickupBall");
    claw.closeClaw();
    Logger.consoleLog();
  }

  @Override
  protected boolean isFinished() {
    return isTimedOut();
  }

  @Override
  protected void end() {
    super.end();
    arm.setHeldItem(HeldItem.BALL);
  }

  @Override
  protected void interrupted() {
    super.interrupted();
    end();
  }

}

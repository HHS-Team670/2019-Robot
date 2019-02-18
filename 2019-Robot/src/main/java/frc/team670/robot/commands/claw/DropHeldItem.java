/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.ConditionalCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Claw;
import frc.team670.robot.subsystems.Arm.HeldItem;

public class DropHeldItem extends ConditionalCommand {
  private Arm arm;

  public DropHeldItem(Claw claw, Arm arm) {
    super(new DropBall(claw, arm), new DropHatch(claw, arm));
    this.arm = arm;
  }

  protected boolean condition() {
    return arm.getHeldItem().equals(HeldItem.BALL);
  }
}

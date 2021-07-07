/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.HeldItem;

/**
 * Add your docs here.
 */
public class ToggleHeldItem extends InstantCommand {
 
  private Arm arm;

  public ToggleHeldItem(Arm arm) {
    super();
    this.arm = arm;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(arm.getHeldItem().equals(HeldItem.NONE)) {
      arm.setHeldItem(HeldItem.BALL);
    }
    else if(arm.getHeldItem().equals(HeldItem.BALL)) {
      arm.setHeldItem(HeldItem.HATCH);
    }
    else {
      arm.setHeldItem(HeldItem.NONE);
    }
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.claw;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.ArmPathGenerator;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.HeldItem;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Claw;

public class YeetHeldItem extends CommandGroup {

  public YeetHeldItem(Claw claw, Arm arm) {
    Command moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.GRAB_BALL_GROUND_BACK), arm);
    addSequential(moveArm);
    if(arm.getHeldItem().equals(HeldItem.BALL)) {
      addSequential(new DropBall(claw, arm));
    } else {
      addSequential(new DropHatch(claw, arm));
    }
  }
}

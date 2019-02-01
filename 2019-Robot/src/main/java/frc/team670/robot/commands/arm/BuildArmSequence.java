/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.cameras.FlipCamera;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;

public class BuildArmSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public BuildArmSequence(String[] armSequence, Arm arm) {
    addSequential(new FlipCamera());

    ArmState destination = Arm.getArmState(LegalState.valueOf(armSequence[0]));
    addSequential(new MoveArm(destination, arm));
    // addSequential: new claw command

  }
}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import frc.team670.robot.commands.arm.movement.MoveElbow;
import frc.team670.robot.commands.arm.movement.MoveExtension;
import frc.team670.robot.commands.arm.movement.MoveWrist;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;

public class NeutralToLowerHatch extends ArmTransition {
  
  public NeutralToLowerHatch(Arm arm) {
    super(LegalState.NEUTRAL, LegalState.NEUTRAL, arm);

    addSequential(new MoveElbow(arm.getElbow(), 30));
    addParallel(new MoveWrist(arm.getWrist(), 40));
    addParallel(new MoveExtension(arm.getExtension(), 6));



    /*
     * Enter your addSequential() and addParallel() commands here.
     */

  }

}

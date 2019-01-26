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

public class NeutralToStow extends ArmTransition {
  
  public NeutralToStow(Arm arm) {
    super(Arm.getArmState(LegalState.NEUTRAL), Arm.getArmState(LegalState.STOW), arm);

    addParallel(new MoveExtension(arm.getExtension(), 50)); //TODO set this
    addParallel(new MoveWrist(arm.getWrist(), 33));
    addSequential(new MoveElbow(arm.getElbow(), 6));


    /*
     * Enter your addSequential() and addParallel() commands here.
     */

  }

}

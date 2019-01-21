/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;

public class NeutralToCargoPickup extends ArmTransition {
  
  public NeutralToCargoPickup() {
    super(Arm.getArmState(LegalState.NEUTRAL), Arm.getArmState(LegalState.NEUTRAL));

    /*
     * Enter your addSequential() and addParallel() commands here.
     */

  }

}

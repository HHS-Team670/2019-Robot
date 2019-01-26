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

/**
 * Add your docs here.
 */
public class NeutralToReadyToClimb extends ArmTransition {
    public NeutralToReadyToClimb (Arm arm){
        super(Arm.getArmState(LegalState.NEUTRAL), Arm.getArmState(LegalState.READY_TO_CLIMB), arm);

    addSequential(new MoveElbow(arm.getElbow(), -45)); // TODO set these
    addParallel(new MoveWrist(arm.getWrist(), 0));
    addParallel(new MoveExtension(arm.getExtension(), 0));
    }
}

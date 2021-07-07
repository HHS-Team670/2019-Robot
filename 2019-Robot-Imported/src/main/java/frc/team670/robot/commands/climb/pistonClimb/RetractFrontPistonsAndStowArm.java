/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team670.robot.commands.arm.movement.MoveArm;

import frc.team670.robot.commands.climb.pistonClimb.RetractFrontPistons;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Climber;

public class RetractFrontPistonsAndStowArm extends CommandGroup {
  /**
   * Retracts the front pistons and puts the arm into Stow positions
   * Cancels the arm climb command so CycleClimb moves onto the next step in the climb sequence
   */
  public RetractFrontPistonsAndStowArm(Arm arm, Climber climber) {
    addParallel(new MoveArm(Arm.getArmState(LegalState.STOW), arm));
    addSequential(new RetractFrontPistons(climber));
    addSequential(new WaitForChildren());
  }
}

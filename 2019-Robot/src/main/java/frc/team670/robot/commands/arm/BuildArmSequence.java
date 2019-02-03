/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.arm.movement.PlaceOrGrab;
import frc.team670.robot.commands.cameras.FlipCamera;
import frc.team670.robot.commands.climb.armClimb.CancelArmClimb;
import frc.team670.robot.commands.intake.RunIntake;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.Robot;

public class BuildArmSequence extends CommandGroup {
  /**
   * Add your docs here.
   */
  public BuildArmSequence(String[] armSequence, Arm arm) {
    for (int i = 0; i < armSequence.length; i++) {
      String position = armSequence[i];
      if (position.toUpperCase().equals(position)) {
        addSequential(new MoveArm(Arm.getArmState(LegalState.valueOf(position)), arm));
      } else if (position.equals("place")) {
        addSequential(new PlaceOrGrab(LegalState.valueOf(armSequence[i-1]), true));
      } else if (position.equals("grab")) {
        addSequential(new PlaceOrGrab(LegalState.valueOf(armSequence[i-1]), false));
      } else if (position.equals("run_intake")) {
        addSequential(new RunIntake(Robot.intake, Robot.sensors));
      } else if (position.equals("next_step_climb")) {
        // TODO: addSequential next step of the climbing process
      } else if (position.equals("cancel_arm_climb")) {
        addSequential(new CancelArmClimb(arm));
      }
    }
  }
}

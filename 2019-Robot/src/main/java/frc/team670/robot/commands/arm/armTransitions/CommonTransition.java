/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.WaitForChildren;
import frc.team670.robot.commands.arm.movement.MoveElbow;
import frc.team670.robot.commands.arm.movement.MoveExtension;
import frc.team670.robot.commands.arm.movement.MoveWrist;
import frc.team670.robot.commands.intake.MoveIntakeToSetpointAngle;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Intake;

/**
 * A common transition that can be used to move the arm straight to any position. Use this if you are not concerned about
 * the arm hitting anything on its path between the start and destination ArmStates
 */
public class CommonTransition extends ArmTransition {
  /**
   * @param start The start ArmState (the ArmState that holds this transitions)
   * @param destination The ending ArmState (where this transition should go to)
   * @param arm The Arm. Should be the static instance of Arm held in Robot
   * @param intake The Intake. Should be the static instance of Intake held in Robot
   */
  public CommonTransition(LegalState start, LegalState destination, Arm arm, BaseIntake intake) {
    super(start, destination, arm, intake);
  }

  @Override
  public void initTransition() {

    ArmState dest = getDest();
    if(dest.isIntakeDeployed()) {
      addParallel(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, intake));
    } else {
      addParallel(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, intake));
    }
    addParallel(new MoveWrist(wrist, dest.getWristAngle()));
    addParallel(new MoveExtension(extension, dest.getExtensionLength()));
    addSequential(new MoveElbow(elbow, dest.getElbowAngle()));
    addSequential(new WaitForChildren());
  }

}

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
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Intake;

public class LowerHatchToNeutral extends ArmTransition {
  /**
   * Add your docs here.
   */
  public LowerHatchToNeutral(Arm arm, BaseIntake intake) {
    super(LegalState.PLACE_HATCH_ROCKET_LOW_FORWARD, LegalState.NEUTRAL, arm, intake);
  }

  @Override
  public void initTransition() {
    ArmState dest = getDest();
    // You would have more Move Commands here to follow a specific path, then end at the destination
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

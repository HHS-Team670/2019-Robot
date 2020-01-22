/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.armTransitions;

import edu.wpi.first.wpilibj.command.CommandGroup;
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
import frc.team670.robot.utils.sort.Edge;

/**
 * A common transition that can be used to move the arm straight to any
 * position. Use this if you are not concerned about the arm hitting anything on
 * its path between the start and destination ArmStates
 */
public class CommonTransition extends ArmTransition {
  private Arm arm;
  private LegalState start, destination;
  private BaseIntake intake;

  // Wooden Piece height above the intake
  private static final double INTAKE_STICKING_OUT_HEIGHT = 2.5;

  /**
   * @param start       The start ArmState (the ArmState that holds this
   *                    transitions)
   * @param destination The ending ArmState (where this transition should go to)
   * @param arm         The Arm. Should be the static instance of Arm held in
   *                    Robot
   * @param intake      The Intake. Should be the static instance of Intake held
   *                    in Robot
   */
  public CommonTransition(LegalState start, LegalState destination, Arm arm, BaseIntake intake) {
    super(start, destination, arm, intake);
    this.arm = arm;
    this.start = start;
    this.destination = destination;
    this.intake = intake;
  }

  @Override
  public void initTransition() {
    ArmState dest = getDest();


    addParallel(new MoveWrist(wrist, dest.getWristAngle()));
    addParallel(new MoveExtension(extension, dest.getExtensionLength()));
    addSequential(new MoveElbow(elbow, dest.getElbowAngle()));
    addSequential(new WaitForChildren());

    if (destination.equals(LegalState.NEUTRAL)){
      if (dest.isIntakeDeployed()) {
        addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, intake));
      } else {
        addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, intake));
      }
    }
  }

  @Override
  public CommandGroup getCommand() {
    CommonTransition commonTransitionCommand = new CommonTransition(start, destination, arm, intake);
    commonTransitionCommand.initTransition();
    return commonTransitionCommand;
  }

  @Override
  public int compareTo(Edge o) {
    return (this == o) ? 0 : 1;
  }

}

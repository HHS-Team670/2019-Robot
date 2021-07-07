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
import frc.team670.robot.commands.claw.CloseClaw;
import frc.team670.robot.commands.claw.OpenClaw;
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
public class NeutralToGrabBallIntake extends ArmTransition {
  private Arm arm;
  private LegalState start, destination;
  private BaseIntake intake;

  /**
   * @param start       The start ArmState (the ArmState that holds this
   *                    transitions)
   * @param destination The ending ArmState (where this transition should go to)
   * @param arm         The Arm. Should be the static instance of Arm held in
   *                    Robot
   * @param intake      The Intake. Should be the static instance of Intake held
   *                    in Robot
   */
  public NeutralToGrabBallIntake(Arm arm, BaseIntake intake) {
    super(LegalState.NEUTRAL, LegalState.GRAB_BALL_INTAKE, arm, intake);
    this.arm = arm;
    this.intake = intake;
    setInterruptible(false);
  }

  @Override
  public void initTransition() {
    addParallel(new MoveExtension(extension, 2));
    addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED, intake));
    addSequential(new WaitForChildren());
    if(arm.getClaw().isOpen()){
      addParallel(new CloseClaw(arm.getClaw()));
    }

    addParallel(new MoveWrist(wrist, Arm.getStates().get(LegalState.GRAB_BALL_INTAKE).getWristAngle()));
    addParallel(new MoveElbow(elbow, Arm.getStates().get(LegalState.GRAB_BALL_INTAKE).getElbowAngle()));
    addSequential(new WaitForChildren());
    addSequential(new MoveExtension(extension, Arm.getStates().get(LegalState.GRAB_BALL_INTAKE).getExtensionLength()));
    addSequential(new OpenClaw(arm.getClaw()));
  }

  @Override
  public CommandGroup getCommand() {
    NeutralToGrabBallIntake command = new NeutralToGrabBallIntake(arm, intake);
    command.initTransition();
    return command;
  }

  @Override
  public int compareTo(Edge o) {
    return (this == o) ? 0 : 1;
  }

}

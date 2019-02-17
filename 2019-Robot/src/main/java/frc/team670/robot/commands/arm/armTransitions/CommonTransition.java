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
    ArmState source = getSource();
    ArmState dest = getDest();

    double intakeHighPoint = Intake.INTAKE_FIXED_LENGTH_IN_INCHES + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES;
    double intakeQuarterPoint = Intake.INTAKE_FIXED_LENGTH_IN_INCHES
        + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(Math.toRadians(45));

    double intakeSourceY = intake.getIntakeCoordinates().getY();
    double intakeDestY = (dest.isIntakeDeployed())
        ? (Intake.INTAKE_FIXED_LENGTH_IN_INCHES
            + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(Intake.INTAKE_ANGLE_DEPLOYED))
        : (Intake.INTAKE_FIXED_LENGTH_IN_INCHES
            + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(Intake.INTAKE_ANGLE_IN));
    double sourceY = source.getMaximumLowestPointOnClaw();
    double destY = dest.getMaximumLowestPointOnClaw();
    double sourceX = source.getCoordPosition().getX();
    double destX = dest.getCoordPosition().getX();

    boolean moveIntakeSecond = false;
    boolean intakeMovementsAddedSequential = false;

    double currentArmX = Arm
        .getCoordPosition(elbow.getAngleInDegrees(), wrist.getAngleInDegrees(), extension.getLengthInches()).getX();
    double currentArmY = Arm.getCurrentLowestPointOnArm(elbow.getAngleInDegrees(), wrist.getAngleInDegrees(),
        extension.getLengthInches());

    // Only reason why intake would have to move second is if the movement of the
    // intake would hit the arm

    // If intake has to move from in to deployed or from deployed to in
    // if (source.isIntakeDeployed() != dest.isIntakeDeployed()) {
    // //If a point along the intake's trajectory is in between the Y coordinates of
    // the arm's start and end
    // if ((intakeSourceY > destY && intakeSourceY < sourceY) ||
    // (intakeHighPoint > destY && intakeHighPoint < sourceY) ||
    // (intakeQuarterPoint > destY && intakeQuarterPoint < sourceY) ||
    // (intakeDestY > destY && intakeDestY < sourceY)){
    // moveIntakeSecond = true;
    // }

    // //If a point along the intake's trajectory is in between the Y coordinates of
    // the arm's start and end
    // if ((intakeSourceY > sourceY && intakeSourceY < destY) ||
    // (intakeHighPoint > sourceY && intakeHighPoint < destY) ||
    // (intakeQuarterPoint > sourceY && intakeQuarterPoint < destY) ||
    // (intakeDestY > sourceY && intakeDestY < destY)){
    // moveIntakeSecond = true;
    // }

    // if (intakeHighPoint > currentArmY && currentArmX >
    // intake.getIntakeCoordinates().getX() && currentArmX <
    // intake.getIntakeCoordinates().getX()){
    // moveIntakeSecond = true;
    // }

    // If arm doesn't even come near the intake, then there's no point adding the
    // command sequentially
    // if((intakeHighPoint >= destY && intakeHighPoint <= sourceY) ||
    // (intakeHighPoint >= sourceY && intakeHighPoint <= destY) && (sourceX > 0 ||
    // destX > 0)){
    // intakeMovementsAddedSequential = true;
    // }
    // }

    // // Redundant code most likely, will run here if boolean says to not run
    // second
    // if (!moveIntakeSecond) {
    // if (intakeMovementsAddedSequential) {
    // if (dest.isIntakeDeployed()) {
    // addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED,
    // intake));
    // } else {
    // addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, intake));
    // }
    // } else {
    // if (dest.isIntakeDeployed()) {
    // addParallel(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED,
    // intake));
    // } else {
    // addParallel(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, intake));
    // }
    // }
    // }

    addParallel(new MoveWrist(wrist, dest.getWristAngle()));
    addParallel(new MoveExtension(extension, dest.getExtensionLength()));
    addSequential(new MoveElbow(elbow, dest.getElbowAngle()));
    addSequential(new WaitForChildren());

    // Redundant code most likely, will run here if boolean says to run second
    // if (intakeMovementsAddedSequential) {
    // if (dest.isIntakeDeployed()) {
    // addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_DEPLOYED,
    // intake));
    // } else {
    // addSequential(new MoveIntakeToSetpointAngle(Intake.INTAKE_ANGLE_IN, intake));
    // }
    // }
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

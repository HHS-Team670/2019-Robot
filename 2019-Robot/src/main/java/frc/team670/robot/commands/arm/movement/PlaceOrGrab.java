/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.claw.DropBall;
import frc.team670.robot.commands.claw.DropHatch;
import frc.team670.robot.commands.claw.GrabHatch;
import frc.team670.robot.commands.claw.PickupBall;
import frc.team670.robot.commands.intake.AutoPickupCargo;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.ArmState;
import frc.team670.robot.subsystems.Arm.LegalState;


public class PlaceOrGrab extends CommandGroup {
  private CommandGroup moveArm;

  /**
   * Moves the arm and claw to do the right thing based on the current arm state
   * @param state the current LegalState of the arm
   * @param isPlacing whether or not an object is being placed (true if placing, false if grabbing)
   */
  public PlaceOrGrab(boolean isPlacing) {
    SmartDashboard.putString("current-command", "PlaceOrGrab");
    ArmState state = Arm.getCurrentState();
    if (state.equals(Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_BACK))) {
      addSequential(new DropBall(Robot.claw, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_FORWARD))) {
      addSequential(new DropBall(Robot.claw, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK))) {
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_BACK), Robot.arm);
      addSequential(moveArm);
      addSequential(new DropBall(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD))) {
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD), Robot.arm);
      addSequential(moveArm);
      addSequential(new DropBall(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK))) {
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK), Robot.arm);
      addSequential(moveArm);
      addSequential(new DropBall(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK))) {
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK), Robot.arm);
      addSequential(moveArm);
      addSequential(new DropHatch(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD))) {
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD), Robot.arm);
      addSequential(moveArm);
      addSequential(new DropHatch(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_GROUND_BACK))) {
      addSequential(new PickupBall(Robot.claw, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_INTAKE))) { // check this
      addSequential(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_BACK))) { // figure this out
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_BACK), Robot.arm);
      addSequential(moveArm);
      addSequential(new PickupBall(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD))) { // figure this out
      moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD), Robot.arm);
      addSequential(moveArm);
      addSequential(new PickupBall(Robot.claw, Robot.arm));
      moveArm = ArmPathGenerator.getPath(state, Robot.arm);
      addSequential(moveArm);
    } else if (state.equals(Arm.getArmState(LegalState.READY_LOW_HATCH_BACK))) {
      if (isPlacing) {
        moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.LOW_HATCH_BACK), Robot.arm);
        addSequential(moveArm);
        addSequential(new DropHatch(Robot.claw, Robot.arm));
        moveArm = ArmPathGenerator.getPath(state, Robot.arm);
        addSequential(moveArm);
      } else {
        moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.LOW_HATCH_BACK), Robot.arm);
        addSequential(moveArm);
        addSequential(new GrabHatch(Robot.claw, Robot.arm));
        moveArm = ArmPathGenerator.getPath(state, Robot.arm);
        addSequential(moveArm);
      }
    } else if (state.equals(Arm.getArmState(LegalState.READY_LOW_HATCH_FORWARD))) {
      if (isPlacing) {
        moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.LOW_HATCH_FORWARD), Robot.arm);
        addSequential(moveArm);
        addSequential(new DropHatch(Robot.claw, Robot.arm));
        moveArm = ArmPathGenerator.getPath(state, Robot.arm);
        addSequential(moveArm);
      } else {
        moveArm = ArmPathGenerator.getPath(Arm.getArmState(LegalState.LOW_HATCH_FORWARD), Robot.arm);
        addSequential(moveArm);
        addSequential(new GrabHatch(Robot.claw, Robot.arm));
        moveArm = ArmPathGenerator.getPath(state, Robot.arm);
        addSequential(moveArm);
      }
    }
  }
}

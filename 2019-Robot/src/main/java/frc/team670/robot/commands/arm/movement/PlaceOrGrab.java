/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import edu.wpi.first.wpilibj.command.CommandGroup;
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
  /**
   * Moves the arm and claw to do the right thing based on the current arm state
   * @param state the current LegalState of the arm
   * @param isPlacing whether or not an object is being placed (true if placing, false if grabbing)
   */
  public PlaceOrGrab(boolean isPlacing) {
    ArmState state = Arm.getCurrentState();
    if (state.equals(Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_BACK))) {
      addSequential(new DropBall(Robot.claw));
    } else if (state.equals(Arm.getArmState(LegalState.PLACE_BALL_CARGOSHIP_FORWARD))) {
      addSequential(new DropBall(Robot.claw));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_LOW_BACK))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_BACK), Robot.arm));
      addSequential(new DropBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_LOW_FORWARD))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_LOW_FORWARD), Robot.arm));
      addSequential(new DropBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_BACK))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_MIDDLE_BACK), Robot.arm));
      addSequential(new DropBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_BALL_ROCKET_MIDDLE_FORWARD))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_BALL_ROCKET_MIDDLE_FORWARD), Robot.arm));
      addSequential(new DropBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_BACK))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_BACK), Robot.arm));
      addSequential(new DropHatch(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_PLACE_HATCH_ROCKET_MIDDLE_FORWARD))) {
      addSequential(new MoveArm(Arm.getArmState(LegalState.PLACE_HATCH_ROCKET_MIDDLE_FORWARD), Robot.arm));
      addSequential(new DropHatch(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_GROUND_BACK))) {
      addSequential(new PickupBall(Robot.claw));
    } else if (state.equals(Arm.getArmState(LegalState.GRAB_BALL_INTAKE))) { // check this
      addSequential(new AutoPickupCargo(Robot.arm, Robot.intake, Robot.claw, Robot.sensors));
    } else if (state.equals(Arm.getArmState(LegalState.READY_GRAB_BALL_LOADINGSTATION_BACK))) { // figure this out
      addSequential(new MoveArm(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_BACK), Robot.arm));
      addSequential(new PickupBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_GRAB_BALL_LOADINGSTATION_FORWARD))) { // figure this out
      addSequential(new MoveArm(Arm.getArmState(LegalState.GRAB_BALL_LOADINGSTATION_FORWARD), Robot.arm));
      addSequential(new PickupBall(Robot.claw));
      addSequential(new MoveArm(state, Robot.arm));
    } else if (state.equals(Arm.getArmState(LegalState.READY_LOW_HATCH_BACK))) {
      if (isPlacing) {
        addSequential(new MoveArm(Arm.getArmState(LegalState.LOW_HATCH_BACK), Robot.arm));
        addSequential(new DropHatch(Robot.claw));
        addSequential(new MoveArm(state, Robot.arm));
      } else {
        addSequential(new MoveArm(Arm.getArmState(LegalState.LOW_HATCH_BACK), Robot.arm));
        addSequential(new GrabHatch(Robot.claw));
        addSequential(new MoveArm(state, Robot.arm));
      }
    } else if (state.equals(Arm.getArmState(LegalState.READY_LOW_HATCH_FORWARD))) {
      if (isPlacing) {
        addSequential(new MoveArm(Arm.getArmState(LegalState.LOW_HATCH_FORWARD), Robot.arm));
        addSequential(new DropHatch(Robot.claw));
        addSequential(new MoveArm(state, Robot.arm));
      } else {
        addSequential(new MoveArm(Arm.getArmState(LegalState.LOW_HATCH_FORWARD), Robot.arm));
        addSequential(new GrabHatch(Robot.claw));
        addSequential(new MoveArm(state, Robot.arm));
      }
    }
  }
}

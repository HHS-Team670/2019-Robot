/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.controlClimb;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.climb.armClimb.ArmClimb;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.commands.climb.pistonClimb.RetractBackPistons;
import frc.team670.robot.commands.climb.pistonClimb.RetractFrontPistons;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.Climber;

/**
 * Allows one Button to cycle through all the necessary stages of climbing:
 * deploy pistons, drag the robot with the arm, retract the front pistons, and
 * retract the back pistons.
 */
public class CycleClimb extends InstantCommand {
  private int setPoint;
  private ClimbStage cg;
  private Arm arm;
  private Climber climber;
  private MustangSensors sensors;

  /**
   * @param setPoint The setpoint in encoder ticks corresponding to the height you
   *                 want to climb to. RobotConstants: PISTON_ENCODER_FLAT,
   *                 PISTON_ENCODER_LEVEL_TWO, PISTON_ENCODER_LEVEL_THREE
   */
  public CycleClimb(Arm arm, Climber climber, MustangSensors sensors, int setPoint) {

    this.setPoint = setPoint;
    this.arm = arm;
    this.climber = climber;
    this.sensors = sensors;

    cg = ClimbStage.DEPLOY_PISTONS;
    String value = "";
    if (setPoint == Climber.PISTON_ENCODER_FLAT) {
      value = "FLAT";
    } else if (setPoint == Climber.PISTON_ENCODER_LEVEL_TWO) {
      value = "LEVEL_TWO";
    } else if (setPoint == Climber.PISTON_ENCODER_LEVEL_THREE) {
      value = "LEVEL_THREE";
    }
    SmartDashboard.putString("climb-level", value);
  }

  // Called just before this Command runs the first time

  /**
   * Has an enum which stores what command to run based on what how many times the
   * command has been called. This allows one button to cycle through a set of
   * different commands
   */
  @Override
  protected void initialize() {
    /*
    If robot hasn't retracted the front pistons yet and driver attempts to move onto the next stage, but there's 
    very little time left, this will stow the arm and bring the robot back down to flat
    */
    Scheduler.getInstance().add(new CancelClimbBasedOnTimeLeftInMatch(arm, climber, sensors));

    switch (cg) {
     case DEPLOY_PISTONS:
      CommandGroup initiatePistonClimb = new CommandGroup();
      initiatePistonClimb.addSequential(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), arm));
      initiatePistonClimb.addSequential(new PistonClimbWithTiltControl(setPoint, climber, sensors));
      Scheduler.getInstance().add(initiatePistonClimb);
      cg = ClimbStage.ARM_CLIMB;
      break;
    case ARM_CLIMB:
      if (climber.getFrontControllerOnTarget() && climber.getBackControllerOnTarget()) {
        CommandGroup initiateArmClimb = new CommandGroup();
        initiateArmClimb.addSequential(new MoveArm(Arm.getArmState(LegalState.READY_TO_CLIMB), arm));
        initiateArmClimb.addSequential(new ArmClimb(arm, climber));
        Scheduler.getInstance().add(initiateArmClimb);
      }
      if (!ArmClimb.getUserWishesToStillClimb()) {
        cg = ClimbStage.RETRACT_FRONT_PISTONS_AND_STOW_ARM;
      }
      break;
    case RETRACT_FRONT_PISTONS_AND_STOW_ARM:
      Scheduler.getInstance().add(new RetractFrontPistons(climber));
      if (climber.getFrontPistonsRetracted()) {
        cg = ClimbStage.RETRACT_BACK_PISTONS;
      }
      break;
    case RETRACT_BACK_PISTONS:
      Scheduler.getInstance().add(new RetractBackPistons(climber));
      if(climber.getBackPistonsRetracted()) {
        cg = ClimbStage.DEPLOY_PISTONS;
      }
      break;
    default:
      Scheduler.getInstance().add(new PistonClimbWithTiltControl(setPoint, climber, sensors));
      cg = ClimbStage.DEPLOY_PISTONS;
      break;
    }
    SmartDashboard.putString("climb-state", cg.toString());
  }

  /**
   * An enum to represent the different stages of climbing that the command can
   * call
   * 
   */
  public enum ClimbStage {
    DEPLOY_PISTONS, ARM_CLIMB, RETRACT_FRONT_PISTONS_AND_STOW_ARM, RETRACT_BACK_PISTONS;
  }
}

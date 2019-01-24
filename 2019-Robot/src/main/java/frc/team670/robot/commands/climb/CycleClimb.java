/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.commands.climb.pistonClimb.RetractBackPistons;
import frc.team670.robot.commands.climb.pistonClimb.RetractFrontPistons;
import frc.team670.robot.commands.climb.armClimb.ArmClimb;

/**
 * Allows one Button to cycle through all the necessary stages of climbing:
 * deploy pistons, drag the robot with the arm, retract the front pistons, and retract the back pistons.
 */
public class CycleClimb extends InstantCommand {
  private int setPoint;
  private ClimbStage cg;

  /**
   * @param setPoint The setpoint in encoder ticks corresponding to the height you want to climb to.
   * RobotConstants: PISTON_ENCODER_FLAT, PISTON_ENCODER_LEVEL_TWO, PISTON_ENCODER_LEVEL_THREEE
   */
  public CycleClimb(int setPoint) {
    requires(Robot.climber);
    requires(Robot.arm);
    requires(Robot.elbow);
    requires(Robot.wrist);

    this.setPoint = setPoint;
    cg = ClimbStage.DEPLOY_PISTONS;
  }

  // Called just before this Command runs the first time

  /**
   * Has an enum which stores what command to run based on what how many times the command has been called. 
   * This allows one button to cycle through a set of different commands
   */
  @Override
  protected void initialize() {
    switch (cg) {
    case DEPLOY_PISTONS:
      Scheduler.getInstance().add(new PistonClimbWithTiltControl(setPoint));
      cg = ClimbStage.ARM_CLIMB;
      break;
    case ARM_CLIMB:
      if(Robot.climber.getFrontAndBackControllerOnTarget()) {
          Scheduler.getInstance().add(new ArmClimb());
        if (!ArmClimb.getUserWishesToStillClimb()) {
          cg = ClimbStage.RETRACT_FRONT_PISTONS;
        }
      }
      break;
    case RETRACT_FRONT_PISTONS:
      Scheduler.getInstance().add(new RetractFrontPistons());
      if (Robot.climber.getFrontPistonsRetracted()) {
        cg = ClimbStage.RETRACT_BACK_PISTONS;
      }
      break;
    case RETRACT_BACK_PISTONS:
      Scheduler.getInstance().add(new RetractBackPistons());
      cg = ClimbStage.DEPLOY_PISTONS;
      break;
    default:
      Scheduler.getInstance().add(new PistonClimbWithTiltControl(setPoint));
      cg = ClimbStage.DEPLOY_PISTONS;
      break;
    }
  }


  /**
   * An enum to represent the different stages of climbing that the command can call
   * 
   */
  public enum ClimbStage {
    DEPLOY_PISTONS, ARM_CLIMB, RETRACT_FRONT_PISTONS, RETRACT_BACK_PISTONS;
  }

}

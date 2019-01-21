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
import frc.team670.robot.commands.arm.ArmClimb;
import frc.team670.robot.commands.climb.pistonClimb.PistonClimbWithTiltControl;
import frc.team670.robot.commands.climb.pistonClimb.RetractBackPistons;
import frc.team670.robot.commands.climb.pistonClimb.RetractFrontPistons;

public class CycleClimb extends InstantCommand {
  private int setPoint;
  public static ClimbStage cg;

  public CycleClimb(int setPoint) {
    requires(Robot.climber);
    requires(Robot.arm);
    requires(Robot.elbow);
    requires(Robot.wrist);

    this.setPoint = setPoint;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    switch (cg) {
    case DEPLOY_PISTONS:
      Scheduler.getInstance().add(new PistonClimbWithTiltControl(setPoint));
      cg = ClimbStage.ARM_CLIMB;
      break;
    case ARM_CLIMB:
      Scheduler.getInstance().add(new ArmClimb());
      if (!ArmClimb.canClimb) {
        cg = ClimbStage.RETRACT_FRONT_PISTONS;
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

  public enum ClimbStage {
    DEPLOY_PISTONS, ARM_CLIMB, RETRACT_FRONT_PISTONS, RETRACT_BACK_PISTONS;
  }

}

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.NullPIDOutput;

public class PistonClimbWithTiltControl extends Command {

  private double tiltControllerLowerOutput = 1.0, tiltControllerUpperOutput = 1.5;
  private double tolerance;
  private double P = 0.0, I = 0.0, D = 0.0, F = 0.0;
  private PIDController tiltController;
  private boolean goingUp;

  public PistonClimbWithTiltControl(boolean goingUp) {
    requires(Robot.climber);
    tiltController = new PIDController(P, I, D, F, Robot.sensors.getNavXPitchPIDSource(), new NullPIDOutput());
    this.goingUp = goingUp;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    tiltController.setSetpoint(0);
    tiltController.setAbsoluteTolerance(tolerance);
    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);
    tiltController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (goingUp) {
      if (Robot.sensors.getPitchDouble() < -tolerance) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.PISTON_DEFAULT_MIN_OUTPUT * (1.5 - tiltController.get()),
         RobotConstants.PISTON_DEFAULT_MAX_OUTPUT * tiltController.get());
      } else if (Robot.sensors.getPitchDouble() > tolerance) {
        Robot.climber.setBackPistonOutputRange(RobotConstants.PISTON_DEFAULT_MIN_OUTPUT * (1.5 - tiltController.get()), 
        RobotConstants.PISTON_DEFAULT_MAX_OUTPUT * tiltController.get()); 
      }
    } else {
      if (Robot.sensors.getPitchDouble() > tolerance) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.PISTON_DEFAULT_MIN_OUTPUT* tiltController.get(), 
        RobotConstants.PISTON_DEFAULT_MAX_OUTPUT * (1.5 - tiltController.get()));
      } else if (Robot.sensors.getPitchDouble() < -tolerance) {
        Robot.climber.setBackPistonOutputRange(RobotConstants.PISTON_DEFAULT_MIN_OUTPUT * tiltController.get(), 
        RobotConstants.PISTON_DEFAULT_MAX_OUTPUT * (1.5 - tiltController.get()));
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return tiltController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setFrontPistonOutputRange(Robot.climber.DEFAULT_MIN_OUTPUT, Robot.climber.DEFAULT_MAX_OUTPUT);
    Robot.climber.setBackPistonOutputRange(Robot.climber.DEFAULT_MIN_OUTPUT, Robot.climber.DEFAULT_MAX_OUTPUT);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climber.setFrontPistonOutputRange(Robot.climber.DEFAULT_MIN_OUTPUT, Robot.climber.DEFAULT_MAX_OUTPUT);
    Robot.climber.setBackPistonOutputRange(Robot.climber.DEFAULT_MIN_OUTPUT, Robot.climber.DEFAULT_MAX_OUTPUT);
  }
}

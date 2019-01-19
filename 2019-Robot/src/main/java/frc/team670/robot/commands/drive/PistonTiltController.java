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


public class PistonTiltController extends Command {

  private double navXPitch;
  private double tiltControllerLowerOutput = 0.5, tiltControllerUpperOutput = 1.5;
  private double tolerance;
  private double P = 0.0, I = 0.0, D = 0.0, FF = 0.0;
  private PIDController tiltController;

  public PistonTiltController() {
    requires(Robot.climber);
    tiltController = new PIDController(P, I, D, Robot.sensors.getNavXPitchPIDSource(), Robot.climber.getFrontPistons());
   
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    tiltController.setSetpoint(0);
    tiltController.setAbsoluteTolerance(tolerance);
    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double frontMinOutput =  Robot.climber.getFrontMinOutput();
    double frontMaxOutput =  Robot.climber.getFrontMaxOutput();
    double backMinOutput =  Robot.climber.getBackMinOutput();
    double backMaxOutput =  Robot.climber.getBackMaxOutput();

    if(Robot.sensors.getPitchDouble() > tolerance){
      Robot.climber.getFrontController().setOutputRange(frontMinOutput, tiltController.get() * frontMaxOutput);
    } else if(Robot.sensors.getPitchDouble() < -tolerance){
      Robot.climber.getBackController().setOutputRange(backMinOutput, tiltController.get() * backMaxOutput);
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

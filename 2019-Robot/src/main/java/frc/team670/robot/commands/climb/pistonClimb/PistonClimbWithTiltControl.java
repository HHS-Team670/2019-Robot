/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.climb.pistonClimb;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;

import frc.team670.robot.Robot;
import frc.team670.robot.subsystems.Climber;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.utils.functions.SettingUtils;
import frc.team670.robot.utils.Logger;

/**
 * Class to deploy the pistons down with a some degree of control from the NavX
 * 
 */
public class PistonClimbWithTiltControl extends Command {

  private double tiltControllerLowerOutput = -0.2, tiltControllerUpperOutput = 0.2;
  private double tiltTolerance = 0; // TODO set this
  private double P = 0.1, I = 0.0, D = 0.0, F = 0.0; // TODO set these
  private PIDController tiltController;
  private boolean goingUp;
  private int loggingIterationCounter, setPoint;

  private Climber climber;

  /**
   * @param setPoint the desired end goal of the climber (Flat, Level 2 or Level 3)
   * @param climber the climber upon which this command will be called on
   */
  public PistonClimbWithTiltControl(int setPoint, Climber climber) {
    requires(Robot.climber);
    tiltController = new PIDController(P, I, D, F, Robot.sensors.getNavXPitchPIDSource(), new NullPIDOutput());
    this.setPoint = setPoint;
    this.climber = climber;
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    
    // Scheduler.getInstance().add(new MoveArm()) TODO: move arm so it's  at neutral state on the climb up
    Robot.climber.enableClimberPIDControllers(setPoint);
    tiltController.setSetpoint(0);
    tiltController.setAbsoluteTolerance(tiltTolerance);
    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);
    tiltController.enable();

    goingUp = (setPoint >= Robot.climber.getFrontTalonPositionInTicks());

    Logger.consoleLog("startBackPistonPosition:%s startFrontPistonPosition:%s ", Robot.climber.getBackTalonPositionInTicks(), Robot.climber.getFrontTalonPositionInTicks());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Math.abs(Robot.sensors.getPitchDouble()) - tiltTolerance > 0){
      Robot.climber.handleTilt(goingUp, tiltTolerance, tiltController.get());
    }

    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("currentBackPistonPosition:%s currentFrontPistonPosition:%s tiltControlScalar:%s", Robot.climber.getBackTalonPositionInTicks(), Robot.climber.getFrontTalonPositionInTicks(), tiltController.get());

    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return climber.getFrontControllerOnTarget() && climber.getBackControllerOnTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    SettingUtils.releaseController(tiltController);
    Logger.consoleLog("endBackPistonPosition:%s endFrontPistonPosition:%s ", Robot.climber.getBackTalonPositionInTicks(), Robot.climber.getFrontTalonPositionInTicks());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog("PistonClimbWithTiltControl interrupted");
  }
}

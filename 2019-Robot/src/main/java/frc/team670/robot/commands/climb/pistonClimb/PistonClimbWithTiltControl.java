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
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.utils.functions.SettingUtils;
import frc.team670.robot.utils.Logger;

/**
 * Class to deploy the pistons down with a some degree of control from the NavX
 * 
 */
public class PistonClimbWithTiltControl extends Command {

  private double tiltControllerLowerOutput = 1.0, tiltControllerUpperOutput = 1.5;
  private double P = 0.0, I = 0.0, D = 0.0, F = 0.0;
  private PIDController tiltController;
  private boolean goingUp;
  private int loggingIterationCounter;

  /**
   * @param setPoint the desired end goal of the climber (Level 2 or Level 3)
   */
  public PistonClimbWithTiltControl(int setPoint) {
    if (!Robot.climber.getBackPistonsRetracted() || !Robot.climber.getFrontPistonsRetracted())
      super.cancel();

    requires(Robot.climber);
    tiltController = new PIDController(P, I, D, F, Robot.sensors.getNavXPitchPIDSource(), new NullPIDOutput());
    Robot.climber.enableClimberPIDControllers(setPoint);

    goingUp = setPoint >= Robot.climber.getFrontTalonPosition();
  }

  // Called just before this Command runs the first time
  protected void initialize() {
    tiltController.setSetpoint(0);
    tiltController.setAbsoluteTolerance(RobotConstants.TILT_TOLERANCE);
    tiltController.setOutputRange(tiltControllerLowerOutput, tiltControllerUpperOutput);
    tiltController.enable();

    Logger.consoleLog("startBackPistonPosition:%s startFrontPistonPosition:%s ", Robot.climber.getBackTalonPosition(),
        Robot.climber.getFrontTalonPosition());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Tilt Control

    // For going up
    if (goingUp) {
      // If tipped down (front is down)
      if (Robot.sensors.getPitchDouble() < -RobotConstants.TILT_TOLERANCE) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER,
            RobotConstants.MAXIMUM_PISTON_POWER * tiltController.get());
        Robot.climber.setBackPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER,
            RobotConstants.MAXIMUM_PISTON_POWER * (1.5 - tiltController.get()));

        // If tipped up (front is up)
      } else if (Robot.sensors.getPitchDouble() > RobotConstants.TILT_TOLERANCE) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER,
            RobotConstants.MAXIMUM_PISTON_POWER * (1.5 - tiltController.get()));
        Robot.climber.setBackPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER,
            RobotConstants.MAXIMUM_PISTON_POWER * tiltController.get());
      }
    }

    // For going up
    else {
      // If tipped down (front is down)
      if (Robot.sensors.getPitchDouble() < -RobotConstants.TILT_TOLERANCE) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.LOWERING_PISTON_POWER * (1.5 - tiltController.get()),
            RobotConstants.MINIMUM_PISTON_POWER);
        Robot.climber.setBackPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER * tiltController.get(),
            RobotConstants.MINIMUM_PISTON_POWER);
        // If tipped down (front is up)
      } else if (Robot.sensors.getPitchDouble() > RobotConstants.TILT_TOLERANCE) {
        Robot.climber.setFrontPistonOutputRange(RobotConstants.LOWERING_PISTON_POWER * tiltController.get(),
            RobotConstants.MINIMUM_PISTON_POWER);
        Robot.climber.setBackPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER * (1.5 - tiltController.get()),
            RobotConstants.MINIMUM_PISTON_POWER);
      }
    }

    if (loggingIterationCounter % 7 == 0)
      Logger.consoleLog("currentBackPistonPosition:%s currentFrontPistonPosition:%s tiltControlScalar:%s",
          Robot.climber.getBackTalonPosition(), Robot.climber.getFrontTalonPosition(), tiltController.get());
  
    loggingIterationCounter++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.climber.getFrontController().onTarget() && Robot.climber.getBackController().onTarget());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climber.setFrontPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    Robot.climber.setBackPistonOutputRange(RobotConstants.MAXIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    SettingUtils.releaseController(tiltController);

    Logger.consoleLog("endBackPistonPosition:%s endFrontPistonPosition:%s ", Robot.climber.getBackTalonPosition(),
        Robot.climber.getFrontTalonPosition());
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.climber.setFrontPistonOutputRange(RobotConstants.MINIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    Robot.climber.setBackPistonOutputRange(RobotConstants.MAXIMUM_PISTON_POWER, RobotConstants.MAXIMUM_PISTON_POWER);
    SettingUtils.releaseController(tiltController);

    Logger.consoleLog("PistonClimbWithTiltControl interrupted");
  }
}

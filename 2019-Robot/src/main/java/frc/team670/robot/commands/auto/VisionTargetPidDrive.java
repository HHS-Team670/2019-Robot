/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.SensorCollection_PIDSource;

/**
 * Drives towards the vision target on the field using distance and angle from the raspberry pi vision
 * code to feed a PID loop.
 * @author hanyun, shaylandias, meganchoy, pallavidas
 */
public class VisionTargetPidDrive extends Command {

  private PIDController  distanceController, headingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double degreeTolerance = 0.05; //degrees
  private static final double distanceTolerance = 0.05; //inches
  private double headingControllerLowerBound = -.15, headingControllerUpperBound = .15;
  private double distanceControllerLowerBound = -.7, distanceControllerUpperBound = .7;
  private final double cameraOffset = 2.5; //distance from camera to front of the robot in inches.
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;

  public VisionTargetPidDrive() {

    requires(Robot.driveBase);
    distanceController = new PIDController(P, I, D, F, Robot.visionPi.getDistanceToTarget(), null);

    headingController = new PIDController (P, I, D, F, Robot.visionPi.getAngleToTarget(), null);
    
    headingController.setInputRange(-30.0,  30.0);
    headingController.setOutputRange(headingControllerLowerBound, headingControllerUpperBound);
    headingController.setAbsoluteTolerance(degreeTolerance);
    headingController.setContinuous(false);

    distanceController.setOutputRange(distanceControllerLowerBound, distanceControllerUpperBound);
    distanceController.setAbsoluteTolerance(distanceTolerance);
    distanceController.setContinuous(false);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Logger.consoleLog("Initialized VisionTargetPidDrive");

    headingController.setSetpoint(0);
    distanceController.setSetpoint(0 + cameraOffset);

    executeCount = 0;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double headingOutput = headingController.get();
    double distanceOutput = distanceController.get();

    if(headingOutput >=0) {
      headingOutput += minimumAngleAdjustment;
    } else {
      headingOutput -=  minimumAngleAdjustment;
    }

    double leftSpeed = distanceOutput - headingOutput;
    double rightSpeed = distanceOutput + headingOutput;

    Robot.driveBase.tankDrive(leftSpeed, rightSpeed);
    if (executeCount % 5 == 0) {
      Logger.consoleLog("Executing VisionTargetPidDrive: headingOutput:%s, distanceOutput:%s, leftSpeed:%s, rightSpeed:%s", headingOutput, distanceOutput, leftSpeed, rightSpeed);
    }

    executeCount ++;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distanceController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("Ended VisionTargetPidDrive: headingOutput:%s, distanceOutput:%s", headingController.get(), distanceController.get());
    Robot.driveBase.stop();
    releaseController(distanceController);
    releaseController(headingController);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted VisionTargetPidDrive");
    end();
  }

  /**
   * Disables controller and releases its resources.
   */
  private void releaseController(PIDController controller) {
    controller.disable();
    controller.free();
  }

}

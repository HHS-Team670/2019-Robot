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
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.SettingUtils;


/**
 * Drives towards the vision target on the field using distance and angle from the raspberry pi vision
 * code to feed a PID loop.
 * @author hanyun
 */
public class BasicVisionPidDrive extends Command {

  private PIDController  distanceController, headingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0; //TODO Set the F value
  private static final double DEGREE_TOLERANCE = 0.05; //degrees
  private static final double DISTANCE_TOLERANCE = 0.05; //inches
  private double headingControllerLowerBound = -.15, headingControllerUpperBound = .15;
  private double distanceControllerLowerBound = -.7, distanceControllerUpperBound = .7;
  private final double CAMERA_OFFSET = 2.5; //TODO: define camera offset.
  private int executeCount;
  private final double MINIMUM_ANGLE_ADJUSTMENT = 0.03;

  public BasicVisionPidDrive() {

    requires(Robot.driveBase);
    distanceController = new PIDController(P, I, D, F, Robot.visionPi.getDistanceToWallTarget(), new NullPIDOutput());

    headingController = new PIDController (P, I, D, F, Robot.visionPi.getAngleToWallTarget(), new NullPIDOutput());

    headingController.setInputRange(-30.0,  30.0);
    headingController.setOutputRange(headingControllerLowerBound, headingControllerUpperBound);
    headingController.setAbsoluteTolerance(DEGREE_TOLERANCE);
    headingController.setContinuous(false);

    distanceController.setOutputRange(distanceControllerLowerBound, distanceControllerUpperBound);
    distanceController.setAbsoluteTolerance(DISTANCE_TOLERANCE);
    distanceController.setContinuous(false);



  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Logger.consoleLog("Initialized BasicVisionPidDrive");

    headingController.setSetpoint(0);
    distanceController.setSetpoint(0 + CAMERA_OFFSET);

    executeCount = 0;

    distanceController.enable();
    headingController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double headingOutput = headingController.get();
    double distanceOutput = distanceController.get() * -1;

    if(headingOutput >=0) {
      headingOutput += MINIMUM_ANGLE_ADJUSTMENT;
    } else {
      headingOutput -=  MINIMUM_ANGLE_ADJUSTMENT;
    }

    double leftSpeed = distanceOutput - headingOutput;
    double rightSpeed = distanceOutput + headingOutput;

    Robot.driveBase.tankDrive(leftSpeed, rightSpeed, false);
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
    SettingUtils.releaseController(distanceController);
    SettingUtils.releaseController(headingController);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted VisionTargetPidDrive");
    end();
  }

}
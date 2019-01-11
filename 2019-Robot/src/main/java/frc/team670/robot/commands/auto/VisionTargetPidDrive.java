/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Pose;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangPi.VisionValue_PIDSource;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Drives towards the vision target on the field using distance and angle from the raspberry pi vision
 * code to feed a PID loop.
 * @author hanyun, shaylandias, meganchoy, pallavidas
 */
public class VisionTargetPidDrive extends Command {

  private PIDController  visionDistanceController, visionHeadingController, distanceControllerEncoders;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double degreeTolerance = 0.05; //degrees
  private static final double distanceTolerance = 0.05; //inches
  private double visionHeadingControllerLowerBound = -.15, visionHeadingControllerUpperBound = .15;
  private double visionDistanceControllerLowerBound = -.7, visionDistanceControllerUpperBound = .7;


  private double distanceControllerLowerBound = 0.05, distanceControllerUpperBound = 0.05;

  private final double cameraOffset = 2.5; //distance from camera to front of the robot in inches. TODO set this
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;

  private Pose robotPosition, currentPose;

  public VisionTargetPidDrive() {

    requires(Robot.driveBase);

    // Distance is in positive numbers, so it outputs a negative number as it tries to go to zero.
    // This is offset by having the robot drive the output * -1
    visionDistanceController = new PIDController(P, I, D, F, new VisionAndPose_PIDSource(Robot.visionPi.getDistanceToTarget(), true), null);
    visionHeadingController = new PIDController (P, I, D, F, new VisionAndPose_PIDSource(Robot.visionPi.getAngleToTarget(), false), null);
    
    visionHeadingController.setInputRange(-30.0,  30.0);
    visionHeadingController.setOutputRange(visionHeadingControllerLowerBound, visionHeadingControllerUpperBound);
    visionHeadingController.setAbsoluteTolerance(degreeTolerance);
    visionHeadingController.setContinuous(false);

    visionDistanceController.setOutputRange(visionDistanceControllerLowerBound, visionDistanceControllerUpperBound);
    visionDistanceController.setAbsoluteTolerance(distanceTolerance);
    visionDistanceController.setContinuous(false);    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Pose.resetPoseInputs();

    robotPosition = new Pose();

    Logger.consoleLog("Initialized VisionTargetPidDrive");

    visionHeadingController.setSetpoint(0);
    visionDistanceController.setSetpoint(0 + cameraOffset);

    executeCount = 0;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    robotPosition.update(Robot.driveBase.getLeftEncoder(), Robot.driveBase.getRightEncoder(), Robot.sensors.getYawDouble());

    /** changed output range to insure that the distanceController isn't going into a negative range */
    double distanceOutput = visionDistanceController.get() * -1;
    double headingOutput = visionHeadingController.get();

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
    return visionDistanceController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("Ended VisionTargetPidDrive: headingOutput:%s, distanceOutput:%s", visionHeadingController.get(), visionDistanceController.get());
    Robot.driveBase.stop();
    releaseController(visionDistanceController);
    releaseController(visionHeadingController);
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

  public class VisionAndPose_PIDSource implements PIDSource {

    private VisionValue_PIDSource visionSource;
    private PIDSourceType pidSourceType;
    private Pose storedPose;
    private double targetAngle, targetDistance;
    
    private boolean isDistance;


    public VisionAndPose_PIDSource(VisionValue_PIDSource visionSource, boolean isDistance) {
      this.visionSource = visionSource;
      this.isDistance = isDistance;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return pidSourceType;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
        pidSourceType = pidSource;
    }

    @Override
    public double pidGet() {
      // return distance left, or angle left
      double visionValue = visionSource.pidGet();

      if (MathUtils.doublesEqual(visionValue, RobotConstants.VISION_ERROR_CODE)) { // If vision cannot find a target
        if (isDistance) {
          // This can be made more efficient by calculating this only once. Gets the target Coordinate Values.
          long targetX = storedPose.getPosX() + (int)(Math.cos(Math.toRadians(targetAngle)) * targetDistance);
          long targetY = storedPose.getPosY() + (int)(Math.sin(Math.toRadians(targetAngle)) * targetDistance);

          // Current Coordinate Values
          long currentPoseX = robotPosition.getPosX();
          long currentPoseY = robotPosition.getPosY();

          // Distance from current coordinate values to target coordinate values.
          double distance = MathUtils.findDistance(currentPoseX, currentPoseY, targetX, targetY);

          return distance;
        }
        else { // Needs to return an angle
          double currentAngle = robotPosition.getRobotAngle();
          return targetAngle - currentAngle;
        }
      }
      else {  // if there is no error
        if (isDistance) {
          targetDistance = visionValue;
        } 
        else {
          targetAngle = visionValue;
        }
        storedPose = robotPosition.clone();
        return visionValue;
      } 
    }
  }

}

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

  private final double cameraOffset = 2.5; //distance from camera to front of the robot in inches.
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;

  private Pose robotPosition, currentPose;

  public VisionTargetPidDrive() {

    requires(Robot.driveBase);
    visionDistanceController = new PIDController(P, I, D, F, Robot.visionPi.getDistanceToTarget(), null);

    visionHeadingController = new PIDController (P, I, D, F, Robot.visionPi.getAngleToTarget(), null);

    //distanceControllerEncoders = new PIDController (P, I, D, F, Robot.driveBase.getLeftEncoder(), null);
    
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

    robotPosition = new Pose();

    Logger.consoleLog("Initialized VisionTargetPidDrive");

    visionHeadingController.setSetpoint(0);
    visionDistanceController.setSetpoint(0 + cameraOffset);

    executeCount = 0;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {



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

    robotPosition.update(Robot.driveBase.getLeftEncoder(), Robot.driveBase.getRightEncoder(), Robot.sensors.getYawDouble());

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
    private boolean errorCalled;


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

      
      if (MathUtils.doublesEqual(visionValue, RobotConstants.VISION_ERROR_CODE)) {
        if (isDistance) {

          long lastPoseX = robotPosition.getPosX();
          long lastPoseY = robotPosition.getPosY();
          double lastPoseAngle = robotPosition.getRobotAngle();
          
          robotPosition.update(Robot.driveBase.getLeftEncoder(), Robot.driveBase.getRightEncoder(), Robot.sensors.getYawDouble());
          
          // long newPoseX = pose.getPosX();
          // long newPoseY = pose.getPosY();
          // double newPoseAngle = pose.getRobotAngle();

          long targetX = (long)(visionValue * Math.cos(Math.toRadians(lastPoseAngle)));
          long targetY = (long)(visionValue * Math.sin(Math.toRadians(lastPoseAngle)));

          long difX = targetX - (newPoseX - lastPoseX);
          long difY = targetY - (newPoseY - lastPoseY);

          return Math.sqrt(difX * difX + difY);
        }
        // isAngle being returned
        else {
          return ;
        }
      }
      else {
        // if there is no error
        errorCalled = false;
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

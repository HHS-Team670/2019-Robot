/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangPi.VisionValue_PIDSource;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.dataCollection.Pose;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Drives towards the vision target on the field using distance and angle from the raspberry pi vision
 * code to feed a PID loop.
 * @author hanyun, shaylandias, meganchoy, pallavidas, kabirbatra, varunjoshi
 */
public class VisionTargetPidDrive extends Command {

  private PIDController  visionDistanceController, visionHeadingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double DEGREE_TOLERANCE = 0.05; //degrees
  private static final double DISTANCE_TOLERANCE = 0.05; //inches
  private double visionHeadingControllerLowerBound = -.15, visionHeadingControllerUpperBound = .15;
  private double visionDistanceControllerLowerBound = -.7, visionDistanceControllerUpperBound = .7;

  private final double CAMERA_OFFSET = 2.5; //distance from camera to front of the robot in inches. TODO set this
  private int executeCount;
  private final double MINIMUM_ANGLE_ADJUSTMENT = 0.03;

  private Pose robotPosition;

  public VisionTargetPidDrive() {

    requires(Robot.driveBase);

    // Distance is in positive numbers, so it outputs a negative number as it tries to go to zero.
    // This is offset by having the robot drive the output * -1
    visionDistanceController = new PIDController(P, I, D, F, new VisionAndPose_PIDSource(Robot.visionPi.getDistanceToWallTarget(), true), new NullPIDOutput());
    visionHeadingController = new PIDController (P, I, D, F, new VisionAndPose_PIDSource(Robot.visionPi.getAngleToWallTarget(), false), new NullPIDOutput());
    
    visionHeadingController.setInputRange(-30.0,  30.0);
    visionHeadingController.setOutputRange(visionHeadingControllerLowerBound, visionHeadingControllerUpperBound);
    visionHeadingController.setAbsoluteTolerance(DEGREE_TOLERANCE);
    visionHeadingController.setContinuous(false);

    visionDistanceController.setOutputRange(visionDistanceControllerLowerBound, visionDistanceControllerUpperBound);
    visionDistanceController.setAbsoluteTolerance(DISTANCE_TOLERANCE);
    visionDistanceController.setContinuous(false);    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.sensors.zeroYaw();

    robotPosition = new Pose();

    Logger.consoleLog("Initialized VisionTargetPidDrive");

    visionHeadingController.setSetpoint(0);
    visionDistanceController.setSetpoint(0 - CAMERA_OFFSET);

    executeCount = 0;

    visionDistanceController.enable();
    visionHeadingController.enable();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    robotPosition.update((long)Robot.driveBase.getLeftDIOEncoderPosition(), (long)Robot.driveBase.getRightDIOEncoderPosition(), Robot.sensors.getYawDouble(), Robot.driveBase.getLeftDIOEncoderVelocityTicks(), Robot.driveBase.getRightDIOEncoderVelocityTicks());

    /** changed output range to insure that the distanceController isn't going into a negative range */
    double distanceOutput = visionDistanceController.get() * -1;
    double headingOutput = visionHeadingController.get();

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
    return visionDistanceController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("Ended VisionTargetPidDrive: headingOutput:%s, distanceOutput:%s", visionHeadingController.get(), visionDistanceController.get());
    Robot.driveBase.stop();
    SettingUtils.releaseController(visionDistanceController);
    SettingUtils.releaseController(visionHeadingController);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted VisionTargetPidDrive");
    end();
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

  // Duplicate class???
  // public class VisionAndPose_PIDSource implements PIDSource {

  //   private VisionValue_PIDSource visionSource;
  //   private PIDSourceType pidSourceType;
  //   private Pose storedPose;
  //   private double targetAngle, targetDistance;
    
  //   private boolean isDistance;


  //   public VisionAndPose_PIDSource(VisionValue_PIDSource visionSource, boolean isDistance) {
  //     this.visionSource = visionSource;
  //     this.isDistance = isDistance;
  //   }

  //   @Override
  //   public PIDSourceType getPIDSourceType() {
  //       return pidSourceType;
  //   }

  //   @Override
  //   public void setPIDSourceType(PIDSourceType pidSource) {
  //       pidSourceType = pidSource;
  //   }

  //   @Override
  //   public double pidGet() {
  //     // return distance left, or angle left
  //     double visionValue = visionSource.pidGet();

  //     if (MathUtils.doublesEqual(visionValue, RobotConstants.VISION_ERROR_CODE)) { // If vision cannot find a target
  //       if (isDistance) {
  //         // This can be made more efficient by calculating this only once. Gets the target Coordinate Values.
  //         long targetX = storedPose.getPosX() + (int)(Math.cos(Math.toRadians(targetAngle)) * targetDistance);
  //         long targetY = storedPose.getPosY() + (int)(Math.sin(Math.toRadians(targetAngle)) * targetDistance);

  //         // Current Coordinate Values
  //         long currentPoseX = robotPosition.getPosX();
  //         long currentPoseY = robotPosition.getPosY();

  //         // Distance from current coordinate values to target coordinate values.
  //         double distance = MathUtils.findDistance(currentPoseX, currentPoseY, targetX, targetY);

  //         return distance;
  //       }
  //       else { // Needs to return an angle
  //         double currentAngle = robotPosition.getRobotAngle();
  //         return targetAngle - currentAngle;
  //       }
  //     }
  //     else {  // if there is no error
  //       if (isDistance) {
  //         targetDistance = visionValue;
  //       } 
  //       else {
  //         targetAngle = visionValue;
  //       }
  //       storedPose = robotPosition.clone();
  //       return visionValue;
  //     } 
  //   }
  // }

}

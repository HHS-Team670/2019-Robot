/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangPi.VisionValues;
import frc.team670.robot.dataCollection.NullPIDOutput;
import frc.team670.robot.dataCollection.Pose;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.functions.SettingUtils;
import jaci.pathfinder.Pathfinder;

/**
 * 
 * Implements a PID Drive based on Vision by constantly setting setpoint and
 * driving them using the NavX gyroscope and encoders. Takes into account time
 * for vision data to be sent and continues off the last seen target if the
 * vision can no longer find it.
 * 
 * @author shaylandias
 */
public class AdvancedVisionPIDDrive extends Command {

  private PIDController distanceController, headingController;
  private static final double P = 0.01, I = 0.0, D = 0.0, F = 0.0;
  private static final double degreeTolerance = 0.05; // degrees
  private static final double distanceTolerance = 0.05; // inches
  private double visionHeadingControllerLowerOutput = -.03, visionHeadingControllerUpperOutput = .03;
  private double visionDistanceControllerLowerOutput = -.15, visionDistanceControllerUpperOutput = .15;

  private final double cameraOffset = 12; // distance from camera to front of the robot in inches.
  private int executeCount;
  private final double minimumAngleAdjustment = 0.03;
  private VisionAngleAndDistanceWithPose visionDistanceAndPose;
  private Pose robotPosition;

  public AdvancedVisionPIDDrive() {
    requires(Robot.driveBase);
    distanceController = new PIDController(P, I, D, F,
        new TwoEncoder_PIDSource(Robot.driveBase.getLeftDIOEncoder(), Robot.driveBase.getRightDIOEncoder()), new NullPIDOutput());
    headingController = new PIDController(P, I, D, F, Robot.sensors.getZeroableNavXPIDSource(), new NullPIDOutput());

    headingController.setInputRange(-180.0, 180.0);
    headingController.setOutputRange(visionHeadingControllerLowerOutput, visionHeadingControllerUpperOutput);
    headingController.setAbsoluteTolerance(degreeTolerance);
    headingController.setContinuous(true);

    distanceController.setOutputRange(visionDistanceControllerLowerOutput, visionDistanceControllerUpperOutput);
    distanceController.setAbsoluteTolerance(distanceTolerance);
    distanceController.setContinuous(false);

    // visionDistance = new
    // VisionAndPose_PIDSource(Robot.visionPi.getDistanceToTarget(), true);
    // visionHeading = new
    // VisionAndPose_PIDSource(Robot.visionPi.getAngleToTarget(), false);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.sensors.zeroYaw();

    robotPosition = new Pose();
    Logger.consoleLog("Initialized AdvancedVisionPIDDrive");

    setSetpoints(0, 0);

    if(MathUtils.doublesEqual(Robot.visionPi.getAngleToWallTarget().getEntry(0), RobotConstants.VISION_ERROR_CODE)) {
      Logger.consoleLog("No image found");
      end();
    }

    executeCount = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    robotPosition.update((long) Robot.driveBase.getLeftDIOEncoderPosition(),
        (long) Robot.driveBase.getRightDIOEncoderPosition(), Robot.sensors.getYawDouble(),
        Robot.driveBase.getLeftDIOEncoderVelocityTicks(), Robot.driveBase.getRightDIOEncoderVelocityTicks());

    double[] visionData = visionDistanceAndPose.getAngleAndDistance();

    headingController.setSetpoint(visionData[0]);
    distanceController.setSetpoint(visionData[1]);

    double distanceOutput = -1 * distanceController.get();  // This is negative since the output from distanceController should be negative and we want to drive reverse.
    double headingOutput = headingController.get();

    if (headingOutput >= 0) {
      headingOutput += minimumAngleAdjustment;
    } else {
      headingOutput -= minimumAngleAdjustment;
    }

    double leftSpeed = distanceOutput - headingOutput;
    double rightSpeed = distanceOutput + headingOutput;

    Robot.driveBase.tankDrive(leftSpeed, rightSpeed);
    if (executeCount % 5 == 0) {
      Logger.consoleLog(
          "Executing AdvancedVisionPIDDrive: headingOutput:%s, distanceOutput:%s, leftSpeed:%s, rightSpeed:%s",
          headingOutput, distanceOutput, leftSpeed, rightSpeed);
    }

    executeCount++;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return distanceController.onTarget() && headingController.onTarget();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("AdvancedVisionPIDDrive Ended. DistanceOutput: %s, HeadingOutput: %s", distanceController.get(),
        headingController.get());
    Robot.driveBase.stop();
    SettingUtils.releaseController(headingController);
    SettingUtils.releaseController(distanceController);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("AdvancedVisionPIDDrive Interrupted.");
    end();
  }

  /**
   * Sets the distance and angle setpoints for the two controllers.
   * 
   * @param distance The distance in inches.
   * @param angle    The angle in degrees [-30, 30]
   */
  private void setSetpoints(double distance, double angle) {
    distanceController.setSetpoint(MathUtils.convertInchesToDriveBaseTicks(distance + cameraOffset));
    headingController.setSetpoint(angle);
  }

  private class TwoEncoder_PIDSource implements PIDSource {
    private Encoder left, right;

    private double initialLeft, initialRight;

    private PIDSourceType pidSourceType;

    public TwoEncoder_PIDSource(Encoder left, Encoder right) {
      this.left = left;
      this.right = right;
      initialLeft = left.get();
      initialRight = right.get();
      pidSourceType = PIDSourceType.kDisplacement;
    }

    @Override
    public PIDSourceType getPIDSourceType() {
      return pidSourceType;
    }

    @Override
    public double pidGet() {
      double displacementLeft = left.get() - initialLeft;
      double displacementRight = right.get() - initialRight;
      return MathUtils.convertDriveBaseTicksToInches((displacementLeft + displacementRight) / 2);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
      this.pidSourceType = pidSource;
    }
  }

  public class VisionAngleAndDistanceWithPose {

    private VisionValues visionSource;
    private Pose poseAtVisionLoss, lastTarget;
    private ArrayList<Pose> lastPoses; // ArrayList to contain the last 5 poses for cross-referencing with vision.

    public VisionAngleAndDistanceWithPose(VisionValues visionSource) {
      this.visionSource = visionSource;
    }

    /**
     * Returns the angle and distance the robot needs to drive as a double[] in the
     * format [angle (degrees), distance (inches)]
     */
    public double[] getAngleAndDistance() {
      double targetAngle, targetDistance;
      double visionValue = visionSource.getAngle();
      Pose currentPose = robotPosition.clone();
      Pose poseAtTimeOfVisionData = lastPoses.get(0);

      // Getting the closest robot Pose to the time the vision data was taken.
      lastPoses.add(currentPose);
      double visionTimeStamp = visionSource.getTimeStamp();
      for (int i = 1; i < lastPoses.size(); i++) {
        Pose nPose = lastPoses.get(i);
        if (nPose != null) {
          if (Math.abs(nPose.getTimeOfPose() - visionTimeStamp) < Math
              .abs(poseAtTimeOfVisionData.getTimeOfPose() - visionTimeStamp)) {
            poseAtTimeOfVisionData = nPose;
          }
        }
      }

      if (MathUtils.doublesEqual(visionValue, RobotConstants.VISION_ERROR_CODE)) { // If vision cannot find a target
        // Grabbing the pose from the moment we lost vision if we don't already have
        // one. If it is already not null, we had set it before so we don't want to
        // overwrite it.
        if (poseAtVisionLoss == null) {
          for (int i = lastPoses.size() - 1; i >= 0; i--) {
            Pose p = lastPoses.get(i);
            if (p != null) {
              poseAtVisionLoss = p;
              break;
            }
          }
          if (poseAtVisionLoss == null) { // If we do not have any poses in the array besides the currentPose.
            poseAtVisionLoss = currentPose; // TODO this might need to give an error code that ends the Command (for
                                            // this to be true, we have to have never had vision data, and thus have no
                                            // target).
          }
        }

        // Calculate target angle and target distance.
        targetAngle = calcAngleWithTimeAdjustment(lastTarget.getPosX(), lastTarget.getPosY(), currentPose.getPosX(),
            currentPose.getPosY());
        targetDistance = calcDistanceWithTimeAdjustment(lastTarget.getPosY(), lastTarget.getPosX(),
            currentPose.getPosY(), currentPose.getPosX());
      } else { // if there is no error
        poseAtTimeOfVisionData = null;
        double angle = visionSource.getAngle();
        double distance = visionSource.getDistance();
        double targetX = Math.cos(Math.toRadians(angle)) * distance;
        double targetY = Math.sin(Math.toRadians(angle)) * distance;

        // Calculate target angle and target distance.
        targetAngle = calcAngleWithTimeAdjustment(targetX, targetY, currentPose.getPosX(), currentPose.getPosY());
        targetDistance = calcDistanceWithTimeAdjustment(targetX, targetY, currentPose.getPosX(), currentPose.getPosY());

        // Stores the lastTarget in case we lose it.
        lastTarget = new Pose(((long) (currentPose.getPosX() + targetDistance * Math.cos(Math.toRadians(targetAngle)))),
            ((long) (currentPose.getPosY() + targetDistance * Math.sin(Math.toRadians(targetAngle)))),
            currentPose.getRobotAngle(), Robot.driveBase.getLeftDIOEncoderVelocityTicks(), Robot.driveBase.getRightDIOEncoderVelocityTicks());
      }
      lastPoses.remove(0); // Remove the oldes Pose from lastPoses so we don't accumulate too many.
      return new double[] { targetAngle, targetDistance };
    }

    private double calcAngleWithTimeAdjustment(double xGoal, double yGoal, double xCurrent, double yCurrent) {
      return AdvancedVisionPIDDrive.calcAngleWithTimeAdjustment(xGoal, yGoal, xCurrent, yCurrent);
    }

    private double calcDistanceWithTimeAdjustment(double xGoal, double yGoal, double xCurrent, double yCurrent) {
      return MathUtils.findDistance(xGoal, yGoal, xCurrent, yCurrent);
    }

  }

  public static double calcAngleWithTimeAdjustment(double xGoal, double yGoal, double xCurrent, double yCurrent) {
    return Pathfinder.boundHalfDegrees(-1 * (Math.toDegrees(Math.atan2(yGoal - yCurrent, xGoal - xCurrent)) - 90));

  }

}

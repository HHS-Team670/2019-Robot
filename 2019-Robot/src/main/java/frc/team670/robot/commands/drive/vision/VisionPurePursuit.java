/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.commands.drive.purePursuit.Path;
import frc.team670.robot.commands.drive.purePursuit.PathGenerator;
import frc.team670.robot.commands.drive.purePursuit.PoseEstimator;
import frc.team670.robot.commands.drive.purePursuit.PurePursuit;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.Vector;
import frc.team670.robot.Robot;


/**
 * Starts a Pure Pursuit path based off vision data
 * @deprecated Replaced by VisionPurePursuitV2 which combines this with PurePursuit into one Command
 */
public class VisionPurePursuit extends CommandGroup {

  private static final double MAX_VEL = 7.5, MAX_ACC = 57, MAX_VELK = 4.5; // VELK = Curve Velocity (1-5)
  // NOTE: Everything passed into the PurePursuit algorithm is in inches/s
  // including constants above and output goes straight to the Spark Neos in
  // inches/s as a velocity control - Check PurePursuitTracker Notifier for this
  // output
  private static final double B = 0.9, A = 1 - B, TOLERANCE = 0.001;
  private static final double SPACING = 1; // Spacing Inches

  /**
   * @param isReversed True if using the back-facing camera on the robot to drive
   *                   backwards
   * @param finalAngle The mutable double for the desired final angle. Should be set by this command so a Pivot can finish it from the surrounding CommandGroup.
   */
  public VisionPurePursuit(DriveBase driveBase, MustangCoprocessor coprocessor, MustangSensors sensors,
      double spaceFromTarget, boolean isReversed, MutableDouble finalAngle) {
    super();

    try {
      if(MathUtils.doublesEqual(coprocessor.getVisionValues()[2], RobotConstants.VISION_ERROR_CODE)) {
        SmartDashboard.putString("warnings", "Coprocess Camera Unplugged: Vision Down");
           Logger.consoleLog("Coprocess Camera Unplugged: Vision Down");
        return;
      } 
    } catch(IndexOutOfBoundsException e) {
      return;
    }

    double horizontalAngle = coprocessor.getAngleToWallTarget();
    System.out.println("HorizontalAngle: " + horizontalAngle);
    if (MathUtils.doublesEqual(horizontalAngle, RobotConstants.VISION_ERROR_CODE)) {
      Logger.consoleLog("No Valid Vision Data found, command quit.");
      SmartDashboard.putString("vision-status", "invalid-data");
      SmartDashboard.putString("warnings", "Vision Target Not Found");
      return;
    }

    // double ultrasonicDistance;
    // if (!isReversed) {
    //   ultrasonicDistance = sensors.getFrontUltrasonicDistance();
    // } else {
    //   ultrasonicDistance = sensors.getAdjustedBackUltrasonicDistance();
    // }
    // ultrasonicDistance *= Math.cos(Math.toRadians(horizontalAngle)); // use cosine to get the straight ultrasonic
    //                                                                  // distance not the diagonal one

    double visionDistance = coprocessor.getDistanceToWallTarget();

    double straightDistance = visionDistance;
    // if (MathUtils.doublesEqual(visionDistance, RobotConstants.VISION_ERROR_CODE)) {
    //   straightDistance = ultrasonicDistance;
    // } else {
    //   straightDistance = (visionDistance < ultrasonicDistance) ? visionDistance : ultrasonicDistance;
    // }

    if (straightDistance > 132) { // Distance is too far, must be invalid data.
      Logger.consoleLog("No Valid Vision Data or Ultrasonic Data found, command quit.");
      SmartDashboard.putString("vision-status", "invalid-data");
      return;
    }

    straightDistance = straightDistance - spaceFromTarget;
    if (straightDistance < 0) {
      System.out.println("Too close to target!");
      this.cancel();
      SmartDashboard.putString("vision-status", "error");
      return;
    }

    SmartDashboard.putString("vision-status", "engaged");

    System.out.println("Angle: " + coprocessor.getAngleToWallTarget());
    // horizontal distance - when going forward a positive horizontal distance is
    // right and negative is left
    double horizontalDistance = straightDistance * Math.tan(Math.toRadians(horizontalAngle)); // x = y * tan(theta)
    // double horizontalDistance = -18;
    double partialDistanceY = (straightDistance) * 2.0 / 5.0;


    System.out.println("straightDist: " + straightDistance + ", horizontalDistance: " + horizontalDistance);
    if (isReversed) { // Flip all of these to match our coord system when looking out the back
      straightDistance *= -1;
      horizontalDistance *= -1;
      partialDistanceY *= -1;
    }

    sensors.zeroYaw(); // NEEDS to happen

    PoseEstimator poseEstimator = new PoseEstimator(driveBase, sensors);

    // Make this start with the Pose Estimator and get its position at this poitn for starting coords.
    Vector startPose = poseEstimator.getPose();
    double startX = startPose.x, startY = startPose.y;
    Vector partialDistance = new Vector(startX + horizontalDistance, startY + partialDistanceY);
    Vector endPoint = new Vector(startX + horizontalDistance, startY + (straightDistance));

    PathGenerator generator = new PathGenerator(SPACING);
    generator.setVelocities(MAX_VEL, MAX_ACC, MAX_VELK);
    generator.setSmoothingParameters(A, B, TOLERANCE);
    generator.addPoints(startPose, partialDistance, endPoint); // Right Angle Segment. All of these are negative since we are driving 2018 Robot backwards.
    
    System.out.println(startPose);
    System.out.println(partialDistance);
    System.out.println(endPoint);
    
    Path path = generator.generatePath(isReversed);

    finalAngle.setValue(horizontalAngle);
    PurePursuit command = new PurePursuit(path, driveBase, sensors, poseEstimator, isReversed, finalAngle, straightDistance, horizontalDistance, spaceFromTarget);
    Robot.leds.setVisionData(true);
    addSequential(command);
  }


}

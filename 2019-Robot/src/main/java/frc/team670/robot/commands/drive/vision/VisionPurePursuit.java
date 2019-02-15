/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.vision;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.drive.purePursuit.Path;
import frc.team670.robot.commands.drive.purePursuit.PathGenerator;
import frc.team670.robot.commands.drive.purePursuit.PoseEstimator;
import frc.team670.robot.commands.drive.purePursuit.PurePursuit;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.math.Vector;

/**
 * Starts a Pure Pursuit path based off vision data
 */
public class VisionPurePursuit extends InstantCommand {

  private static final double MAX_VEL = 16, MAX_ACC = 100, MAX_VELK = 1; // VELK = Curve Velocity (1-5)
  // NOTE: Everything passed into the PurePursuit algorithm is in inches/s
  // including constants above and output goes straight to the Spark Neos in
  // inches/s as a velocity control - Check PurePursuitTracker Notifier for this
  // output
  private static final double B = 0.9, A = 1 - B, TOLERANCE = 0.001;

  private PurePursuit command;
  private MustangCoprocessor coprocessor;
  private MustangSensors sensors;
  private DriveBase driveBase;
  private static final double SPACING = 1; // Spacing Inches
  private double spaceFromTarget;
  private boolean isReversed;
  private static Notifier restrictArmMovement;

  /**
   * @param isReversed True if using the back-facing camera on the robot to drive
   *                   backwards
   */
  public VisionPurePursuit(DriveBase driveBase, MustangCoprocessor coprocessor, MustangSensors sensors,
      double spaceFromTarget, boolean isReversed, boolean lowTarget) {
    super();
    this.coprocessor = coprocessor;
    this.sensors = sensors;
    this.driveBase = driveBase;
    this.spaceFromTarget = spaceFromTarget;
    this.isReversed = isReversed;

    coprocessor.setTargetHeight(lowTarget);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    driveBase.initAutonDrive();

    double horizontalAngle = coprocessor.getAngleToWallTarget();
    if (MathUtils.doublesEqual(horizontalAngle, RobotConstants.VISION_ERROR_CODE)) {
      Logger.consoleLog("No Valid Vision Data found, command quit.");
      SmartDashboard.putString("vision-status", "error");
      return;
    }

    double ultrasonicDistance;
    if (!isReversed) {
      ultrasonicDistance = sensors.getFrontUltrasonicDistance(horizontalAngle);
    } else {
      double ultraLeft = sensors.getBackLeftUltrasonicDistance(horizontalAngle);
      double ultraRight = sensors.getBackRightUltrasonicDistance(horizontalAngle);

      ultrasonicDistance = (ultraLeft < ultraRight) ? ultraLeft : ultraRight; // Take the one that is least
    }
    ultrasonicDistance *= Math.cos(Math.toRadians(horizontalAngle)); // use cosine to get the straight ultrasonic
                                                                     // distance not the diagonal one

    double visionDistance = coprocessor.getDistanceToWallTarget();

    double straightDistance;
    if (MathUtils.doublesEqual(visionDistance, RobotConstants.VISION_ERROR_CODE)) {
      straightDistance = ultrasonicDistance;
    } else {
      straightDistance = (visionDistance < ultrasonicDistance) ? visionDistance : ultrasonicDistance;
    }

    if (straightDistance > 132) { // Distance is too far, must be invalid data.
      Logger.consoleLog("No Valid Vision Data or Ultrasonic Data found, command quit.");
      SmartDashboard.putString("vision-status", "");
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
    double horizontalDistance = -38;// straightDistance *
                                    // Math.tan(Math.toRadians(coprocessor.getRealAngleToWallTarget())); // x = y *
                                    // tan(theta)
    double oneEighthTargetY = (straightDistance) * 1.0 / 8.0;

    System.out.println("straightDist: " + straightDistance + ", horizontalDistance: " + horizontalDistance);
    if (isReversed) { // Flip all of these to match our coord system when looking out the back
      straightDistance *= -1;
      horizontalDistance *= -1;
      oneEighthTargetY *= -1;

      coprocessor.setCamera(true);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    } else{
      coprocessor.setCamera(false);
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    sensors.zeroYaw();

    PoseEstimator poseEstimator = new PoseEstimator(driveBase, sensors);

    // Make this start with the Pose Estimator and get its position at this poitn for starting coords.
    Vector startPose = poseEstimator.getPose();
    double startX = startPose.x, startY = startPose.y;
    Vector oneQuarter = new Vector(startX + horizontalDistance, startY + oneEighthTargetY);
    Vector endPoint = new Vector(startX + horizontalDistance, startY + (straightDistance));

    PathGenerator generator = new PathGenerator(SPACING);
    generator.setVelocities(MAX_VEL, MAX_ACC, MAX_VELK);
    generator.setSmoothingParameters(A, B, TOLERANCE);
    generator.addPoints(startPose, oneQuarter, endPoint); // Right Angle Segment. All of these are negative since we are driving 2018 Robot backwards.
    
    System.out.println(startPose);
    System.out.println(oneQuarter);
    System.out.println(endPoint);
    
    Path path = generator.generatePath(isReversed);

    command = new PurePursuit(path, driveBase, sensors, poseEstimator, isReversed);

    if (straightDistance > 60) { // Protect arm until it is time to actually place
      restrictArmMovement = new Notifier(new Runnable() {

        boolean movedToNeutral = false;
        boolean reversed = isReversed;

        public void run() {

          double ultrasonicDistance;
          if(!isReversed) {
            ultrasonicDistance = sensors.getFrontUltrasonicDistance(horizontalAngle);
          }
          else {
            double ultraLeft = sensors.getBackLeftUltrasonicDistance(horizontalAngle);
            double ultraRight = sensors.getBackRightUltrasonicDistance(horizontalAngle);
      
            ultrasonicDistance = (ultraLeft < ultraRight) ? ultraLeft : ultraRight; // Take the one that is least
          }
          ultrasonicDistance *= Math.cos(Math.toRadians(horizontalAngle)); //use cosine to get the straight ultrasonic distance not the diagonal one
      
          if (ultrasonicDistance < 48) {
            Scheduler.getInstance().add(new MoveArm(Arm.getCurrentState(), Robot.arm));
            cancel();
          } else {
            if(!movedToNeutral) {
              Scheduler.getInstance().add(new MoveArm(Arm.getArmState(LegalState.NEUTRAL), Robot.arm));
              movedToNeutral = true;
            }
          }
        }
      });
      restrictArmMovement.startPeriodic(0.03);
    } else {
      restrictArmMovement = null;
    }

    Scheduler.getInstance().add(command);
  }

  public static void disableArmRestriction() {
    if (restrictArmMovement != null)
      restrictArmMovement.stop();
  }

}

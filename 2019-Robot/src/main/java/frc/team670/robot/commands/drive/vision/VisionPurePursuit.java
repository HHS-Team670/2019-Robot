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
import frc.team670.robot.Robot;
import frc.team670.robot.commands.arm.movement.CancelArmMovement;
import frc.team670.robot.commands.arm.movement.MoveArm;
import frc.team670.robot.commands.drive.purePursuit.Path;
import frc.team670.robot.commands.drive.purePursuit.PathGenerator;
import frc.team670.robot.commands.drive.purePursuit.PoseEstimator;
import frc.team670.robot.commands.drive.purePursuit.PurePursuit;
import frc.team670.robot.dataCollection.MustangCoprocessor;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.subsystems.Arm.LegalState;
import frc.team670.robot.utils.math.Vector;

/**
 * Starts a Pure Pursuit path based off vision data
 */
public class VisionPurePursuit extends InstantCommand {
  
  private static final double MAX_VEL = 16, MAX_ACC = 100, MAX_VELK = 1; // VELK = Curve Velocity (1-5)
  //NOTE: Everything passed into the PurePursuit algorithm is in inches/s including constants above and output goes straight to the Spark Neos in inches/s as a velocity control - Check PurePursuitTracker Notifier for this output
  private static final double B = 0.9, A = 1 - B, TOLERANCE = 0.001;

  private PurePursuit command;
  private MustangCoprocessor coprocessor;
  private MustangSensors sensors;
  private DriveBase driveBase;
  private static final double SPACING = 1; // Spacing Inches
  private double spaceFromTarget;
  private boolean isReversed;
  private static Notifier restrictArmMovement;
  private double straightDistance;


  /**
   * @param isReversed True if using the back-facing camera on the robot to drive backwards
   */
  public VisionPurePursuit(DriveBase driveBase, MustangCoprocessor coprocessor, MustangSensors sensors, double spaceFromTarget, boolean isReversed) {
    super();
    this.coprocessor = coprocessor;
    this.sensors = sensors;
    this.driveBase = driveBase;
    this.spaceFromTarget = spaceFromTarget;
    this.isReversed = isReversed;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    driveBase.initAutonDrive();

    

    double horizontalAngle = coprocessor.getAngleToWallTarget();
    double ultrasonicDistance = sensors.getUltrasonicDistance()*Math.cos(Math.toRadians(horizontalAngle)); //use cosine to get the straight ultrasonic distance not the diagonal one
    double visionDistance = coprocessor.getDistanceToWallTarget();

    straightDistance = 120;// ultrasonicDistance; //(!MathUtils.doublesEqual(visionDistance, RoboConstants.VISION_ERROR_CODE) && visionDistance < ultrasonicDistance) ? visionDistance : ultrasonicDistance;
    // straightDistance = visionDistance;
    straightDistance = straightDistance - spaceFromTarget;

    if (straightDistance > 60) {
      restrictArmMovement = new Notifier(new Runnable() {



        public void run() {
          straightDistance = sensors.getUltrasonicDistance() * Math.cos(Math.toRadians(horizontalAngle));
          if (straightDistance < 48) {
            Scheduler.getInstance().add(new MoveArm(Arm.getCurrentState(), Robot.arm));
            cancel();
          }
        }
      });

      restrictArmMovement.startPeriodic(0.02);
    } else {
      restrictArmMovement = null;
    }

    if(straightDistance < 0){
      System.out.println("Too close to target!");
      this.cancel();
      return;
    }

    System.out.println("Angle: " + coprocessor.getAngleToWallTarget());
    //horizontal distance - when going forward a positive horizontal distance is right and negative is left
    double horizontalDistance = -38;// straightDistance * Math.tan(Math.toRadians(coprocessor.getRealAngleToWallTarget())); // x = y * tan(theta)
    double oneQuarterTargetY = (straightDistance) * 7.0/8.0;


    System.out.println("straightDist: " + straightDistance + ", horizontalDistance: " + horizontalDistance);
    if(isReversed) { // Flip all of these to match our coord system when looking out the back
      straightDistance *= -1;
      horizontalDistance *= -1;
      oneQuarterTargetY *= -1;
    }

    sensors.zeroYaw();

    PoseEstimator poseEstimator = new PoseEstimator(driveBase, sensors);

    // Make this start with the Pose Estimator and get its position at this poitn for starting coords.
    Vector startPose = poseEstimator.getPose();
    double startX = startPose.x, startY = startPose.y;
    Vector oneQuarter = new Vector(startX + horizontalDistance, startY + oneQuarterTargetY);
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

    Scheduler.getInstance().add(command);
  }

  public static void disableArmRestriction() {
    if (restrictArmMovement != null)
      restrictArmMovement.stop();
  }

}

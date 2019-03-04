/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.purePursuit;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.MutableDouble;
import frc.team670.robot.utils.math.DrivePower;

public class PurePursuit extends Command {

  private static final double LOOKAHEAD_DISTANCE_AT_66_INCHES = 15;

  private PurePursuitTracker purePursuitTracker;
  private PoseEstimator poseEstimator;
  private DriveBase driveBase;
  private MustangSensors sensors;
  private int executeCount;
  private MutableDouble finalAngle;
  private double yDistance;
  private double xDistance;
  private double offset;

  /**
   * @param finalAngle a MutableDouble object reference to the angle (using zeroed yaw) this PurePursuit command should end up at compared to the zeroed yaw.
   */
  public PurePursuit(Path path, DriveBase driveBase, MustangSensors sensors, PoseEstimator estimator, boolean isReversed, MutableDouble finalAngle, double yDistance, double xDistance, double offset) {
   this.driveBase = driveBase;
   this.sensors = sensors;
   this.poseEstimator = estimator;
   this.finalAngle = finalAngle;

   this.yDistance = yDistance;
   this.xDistance = xDistance;
   this.offset = offset;
  
   purePursuitTracker = new PurePursuitTracker(poseEstimator, driveBase, sensors, isReversed);
   purePursuitTracker.setPath(path, LOOKAHEAD_DISTANCE_AT_66_INCHES * yDistance/66);
   requires(driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    driveBase.initBrakeMode();
    sensors.zeroYaw();
    purePursuitTracker.reset();
    Logger.consoleLog();
    executeCount = 0;
    // purePursuitTracker.startNotifier(0.01); //Pass in period in seconds
    System.out.println("Start, Pose: " + poseEstimator.getPose());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    poseEstimator.update();
    DrivePower drivePower;

    drivePower = purePursuitTracker.update(poseEstimator.getPose(), driveBase.getLeftMustangEncoderVelocityInInchesPerSecond(), driveBase.getRightMustangEncoderVelocityInInchesPerSecond(), sensors.getRotationAngle().radians());
  
    driveBase.tankDrive(drivePower.getLeft()/60, drivePower.getRight()/60); //Returns in inches/s
    if(executeCount % 5 == 0)
      Logger.consoleLog("Powers (inches): leftPower: %s, rightPower: %s, Pose: %s", drivePower.getLeft(), drivePower.getRight(), poseEstimator.getPose());
    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return purePursuitTracker.isDone();// || Robot.sensors.getUltrasonicDistance() < 15;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Logger.consoleLog("Pose: %s ", poseEstimator.getPose());
    // VisionPurePursuit.disableArmRestriction();
    driveBase.setSparkVelocityControl(0,0);
    double xOffset = xDistance - poseEstimator.getPose().x;
    double yOffset = yDistance + offset - poseEstimator.getPose().y;
    double angle = Math.atan(yOffset/xOffset);
    finalAngle.setValue(finalAngle.getValue()-sensors.getYawDouble()-Math.toDegrees(angle));
    // purePursuitTracker.stopNotifier();
    purePursuitTracker.reset();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    Logger.consoleLog();
  }
}

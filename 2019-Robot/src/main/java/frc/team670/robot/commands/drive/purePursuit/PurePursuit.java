/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.drive.purePursuit;

import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.commands.drive.vision.VisionPurePursuit;
import frc.team670.robot.dataCollection.MustangSensors;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;

public class PurePursuit extends Command {

  private static final double LOOKAHEAD_DISTANCE = 12;

  private PurePursuitTracker purePursuitTracker;
  private PoseEstimator poseEstimator;
  private DriveBase driveBase;
  private MustangSensors sensors;
  private int executeCount;

  public PurePursuit(Path path, DriveBase driveBase, MustangSensors sensors, PoseEstimator estimator, boolean isReversed) {
   this.driveBase = driveBase;
   this.sensors = sensors;
   this.poseEstimator = estimator;
  
   purePursuitTracker = new PurePursuitTracker(poseEstimator, driveBase, sensors, isReversed);
   purePursuitTracker.setPath(path, LOOKAHEAD_DISTANCE);
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
    purePursuitTracker.startNotifier(0.01); //Pass in period in seconds
    System.out.println("Start, Pose: " + poseEstimator.getPose());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //FOLLOWING MOVED TO PurePursuitTracker as a Notifier

    // poseEstimator.update();
    // DrivePower drivePower;

    // drivePower = purePursuitTracker.update(poseEstimator.getPose(), MathUtils.convertDriveBaseTicksToInches(driveBase.getLeftVelocity()), MathUtils.convertDriveBaseTicksToInches(driveBase.getRightVelocity()), sensors.getRotationAngle().radians());
    // driveBase.velocityControl(MathUtils.convertInchesToDriveBaseTicks(drivePower.getLeft()), MathUtils.convertInchesToDriveBaseTicks(drivePower.getRight()));
  
    // if(executeCount % 5 == 0)
    //   Logger.consoleLog("Powers (inches): leftPower: %s, rightPower: %s", drivePower.getLeft(), drivePower.getRight());
    // executeCount++;
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
    VisionPurePursuit.disableArmRestriction();
    driveBase.setSparkVelocityControl(0,0);
    purePursuitTracker.stopNotifier();
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

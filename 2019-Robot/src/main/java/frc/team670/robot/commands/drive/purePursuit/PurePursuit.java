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
import frc.team670.robot.utils.math.DrivePower;

public class PurePursuit extends Command {

  private static final double LOOKAHEAD_DISTANCE = 15;

  private PurePursuitTracker purePursuitTracker;
  private PoseEstimator poseEstimator;
  private DriveBase driveBase;
  private MustangSensors sensors;

  public PurePursuit(Path path, DriveBase driveBase, MustangSensors sensors) {
   this.driveBase = driveBase;
   poseEstimator = new PoseEstimator(driveBase, sensors);
   purePursuitTracker = new PurePursuitTracker(poseEstimator);
   purePursuitTracker.setPath(path, LOOKAHEAD_DISTANCE);
   requires(driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    sensors.zeroYaw();
    poseEstimator.reset();
    purePursuitTracker.reset();
    Logger.consoleLog();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    poseEstimator.update();
    DrivePower drivePower = purePursuitTracker.update(poseEstimator.getPose(), driveBase.getLeftMustangEncoderVelocityInInchesPerSecond(), driveBase.getRightMustangEncoderVelocityInInchesPerSecond(), sensors.getRotationAngle().radians());
    driveBase.setSparkVelocityControl(drivePower.getLeft(), drivePower.getRight());
    Logger.consoleLog("leftPower: %s, rightPower: %s", drivePower.getLeft(), drivePower.getRight());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return purePursuitTracker.isDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    driveBase.setSparkVelocityControl(0, 0);;
    poseEstimator.reset();
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

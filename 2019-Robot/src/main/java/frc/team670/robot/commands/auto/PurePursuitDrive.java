/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.command.Command;
import frc.team254.lib.util.control.Lookahead;
import frc.team254.lib.util.control.Path;
import frc.team254.lib.util.control.PathFollower;
import frc.team254.lib.util.math.RigidTransform2d;
import frc.team254.lib.util.math.Twist2d;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;

public class PurePursuitDrive extends Command {

  private PathFollower pathFollower;

  public PurePursuitDrive(Path path, boolean reversed) {
    requires(Robot.driveBase);
    pathFollower = new PathFollower(path, reversed,
                    new PathFollower.Parameters(
                            new Lookahead(RobotConstants.kMinLookAhead, RobotConstants.kMaxLookAhead,
                                    RobotConstants.kMinLookAheadSpeed, RobotConstants.kMaxLookAheadSpeed),
                            RobotConstants.kInertiaSteeringGain, RobotConstants.kPathFollowingProfileKp,
                            RobotConstants.kPathFollowingProfileKi, RobotConstants.kPathFollowingProfileKv,
                            RobotConstants.kPathFollowingProfileKffv, RobotConstants.kPathFollowingProfileKffa,
                            RobotConstants.kPathFollowingMaxVel, RobotConstants.kPathFollowingMaxAccel,
                            RobotConstants.kPathFollowingGoalPosTolerance, RobotConstants.kPathFollowingGoalVelTolerance,
                            RobotConstants.kPathStopSteeringDistance));
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // RigidTransform2d robot_pose = null; //mRobotState.getLatestFieldToVehicle().getValue();

    // double timestamp = System.currentTimeMillis();

    // Twist2d command = pathFollower.update(timestamp, robot_pose,
    //         RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
    // if (!mPathFollower.isFinished()) {
    //     Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
    //     Robot.driveBase.setVelocityControl(setpoint.left, setpoint.right);
    // } else {
    //   Robot.driveBase.setVelocityControl(0, 0);
    // }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}

package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.subsystems.DriveBase;


/**
 * Pivots the robot in a circle ten times then outputs 
 */
public class MeasureTrackwidth extends Command {

  private final double totalAngleToDrive = -10 * 360;
  private int initialLeftEncoder, initialRightEncoder;
  private double initialAngle, goalAngle;

  public MeasureTrackwidth() {
    requires(Robot.driveBase);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    initialLeftEncoder = Robot.driveBase.getLeftMustangEncoderPositionInTicks();
    initialRightEncoder = Robot.driveBase.getRightMustangEncoderPositionInTicks();
    initialAngle = Robot.sensors.getYawDouble();
    goalAngle = initialAngle + totalAngleToDrive;
    Logger.consoleLog("Initial Angle: " + initialAngle);
    Logger.consoleLog("Initial Left Encoder: " + initialLeftEncoder);
    Logger.consoleLog("Initial Right Encoder: " + initialRightEncoder);
    Logger.consoleLog("Goal Angle: " + goalAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.driveBase.tankDrive(-0.3, 0.3);
    Logger.consoleLog("Current Angle: " + Robot.sensors.getAngle() + "");
    Logger.consoleLog("Goal Angle: " + goalAngle + "");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.sensors.getAngle() < goalAngle;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    double overShoot = Robot.sensors.getYawDouble() - initialAngle;
    double leftDistance = Robot.driveBase.getLeftMustangEncoderPositionInTicks() - initialLeftEncoder;
    double rightDistance = Robot.driveBase.getRightMustangEncoderPositionInTicks() - initialRightEncoder;

    Logger.consoleLog("Spun the Robot 10 times, or 3600 degrees with " + overShoot + " degrees of overshoot");
    Logger.consoleLog("Left Side Distance Driven (Ticks): " + leftDistance);
    Logger.consoleLog("Left Side Distance Driven (Inches): " + DriveBase.convertDriveBaseTicksToInches(leftDistance));
    Logger.consoleLog("Right Side Distance Driven (Ticks): " + rightDistance);
    Logger.consoleLog("Right Side Distance Driven (Inches): " + DriveBase.convertDriveBaseTicksToInches(rightDistance));
    
    double leftRadiusInches = DriveBase.convertDriveBaseTicksToInches(leftDistance) / (10 * 2 * Math.PI);
    double rightRadiusInches = DriveBase.convertDriveBaseTicksToInches(rightDistance) / (10 * 2 * Math.PI);

    double leftDiameter = Math.abs(leftRadiusInches * 2);
    double rightDiameter = Math.abs(rightRadiusInches * 2);

    Logger.consoleLog("Left Diameter Calculation (inches): " + leftDiameter);
    Logger.consoleLog("Right Diameter Calculation (inches): " + rightDiameter);
    Logger.consoleLog("Average Diameter (inches): " + ((leftDiameter + rightDiameter)/2));
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted Measure Wheelbase");
  }
}
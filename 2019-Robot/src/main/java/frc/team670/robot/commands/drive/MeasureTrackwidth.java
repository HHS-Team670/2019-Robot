package frc.team670.robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.Robot;

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
    initialLeftEncoder = Robot.driveBase.getLeftDIOEncoderPosition();
    initialRightEncoder = Robot.driveBase.getRightDIOEncoderPosition();
    initialAngle = Robot.sensors.getYawDouble();
    goalAngle = initialAngle + totalAngleToDrive;
    System.out.println("Initial Angle: " + initialAngle);
    System.out.println("Initial Left Encoder: " + initialLeftEncoder);
    System.out.println("Initial Right Encoder: " + initialRightEncoder);
    System.out.println("Goal Angle: " + goalAngle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // Robot.driveBase.tankDrive(-0.6, 0.6);
    Robot.driveBase.curvatureDrive(0.2, -0.5, true);
    SmartDashboard.putString("Current Angle", Robot.sensors.getYawDouble() + "");
    SmartDashboard.putString("Goal Angle", goalAngle + "");
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.sensors.getYawDouble() < goalAngle;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {

    double overShoot = Robot.sensors.getYawDouble() - initialAngle;
    double leftDistance = Robot.driveBase.getLeftDIOEncoderPosition() - initialLeftEncoder;
    double rightDistance = Robot.driveBase.getRightDIOEncoderPosition() - initialRightEncoder;

    System.out.println("Spun the Robot 10 times, or 3600 degrees with " + overShoot + " degrees of overshoot");
    System.out.println("Left Side Distance Driven (Ticks): " + leftDistance);
    System.out.println("Left Side Distance Driven (Inches): " + (leftDistance / RobotConstants.DIO_TICKS_PER_INCH));
    System.out.println("Right Side Distance Driven (Ticks): " + rightDistance);
    System.out.println("Right Side Distance Driven (Inches): " + (rightDistance / RobotConstants.DIO_TICKS_PER_INCH));
    
    double leftRadiusInches = (leftDistance / RobotConstants.DIO_TICKS_PER_INCH) / (10 * 2 * Math.PI);
    double rightRadiusInches = (rightDistance / RobotConstants.DIO_TICKS_PER_INCH) / (10 * 2 * Math.PI);

    double leftDiameter = leftRadiusInches * 2;
    double rightDiameter = rightRadiusInches * 2;

    System.out.println("Left Diameter Calculation (inches): " + leftDiameter);
    System.out.println("Right Diameter Calculation (inches): " + rightDiameter);
    System.out.println("Average Diameter (inches): " + ((leftDiameter + rightDiameter)/2));
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    System.out.println("Interrupted Measure Wheelbase");
  }
}
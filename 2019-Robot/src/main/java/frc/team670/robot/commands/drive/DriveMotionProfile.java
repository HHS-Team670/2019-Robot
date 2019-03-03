
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.team670.robot.commands.drive;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.subsystems.DriveBase;
import frc.team670.robot.utils.Logger;
import frc.team670.robot.utils.functions.MathUtils;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/**
 * Testing the code with the robot notes:
 *  Robot thinks forward is 180 degrees
 *  Don't use stupid waypoints because code crashes if Pathfinder fails
 */

public class DriveMotionProfile extends Command {

  private Waypoint[] waypoints = new Waypoint[]{new Waypoint(0, 0, 0)};
  private Trajectory.Config config;
  // private Trajectory trajectory;
  private Trajectory leftTrajectory, rightTrajectory;
  private TankModifier modifier;
  private EncoderFollower left, right;
  // Max Velocities in m/s. For generation, we need it to be lower than the actual max velocity, 
  // otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn properly.
  private static final String BASE_PATH_NAME = "home/deploy/";
  private static final double MAX_VELOCITY = 90, MAX_ACCELERATION = 20, MAX_JERK = 60; // Equivalent units in inches
  private static final double TIME_STEP = 0.05;
  // Making this constant higher helps prevent the robot from overturning (overturning also will make it not drive far enough in the correct direction)
  private static final double ANGLE_DIVIDE_CONSTANT = 240.0; // Default = 80


  private static double P = 0.01, I = 0, D = 0.001, KA= 0;

  // Values for logging purposes
  private final int EXECUTE_LOG_INTERVAL = 8;
  private long executeCount;
  private boolean isReversed;

  int initialLeftEncoder, initialRightEncoder;
  
  /**
   * Drives a Pathfinder Motion Profile using set Waypoints
   */
  public DriveMotionProfile(Waypoint[] waypoints, boolean isReversed) {
    requires(Robot.driveBase);

    this.isReversed = isReversed;

    this.waypoints = waypoints.clone();

    config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
    Trajectory trajectory = Pathfinder.generate(waypoints, config);
    modifier = new TankModifier(trajectory).modify(RobotConstants.WHEEL_BASE);
    leftTrajectory = modifier.getLeftTrajectory();
    rightTrajectory = modifier.getRightTrajectory();

     // TODO In the future maybe do this in initialize so when the Command is rerun it starts over for testing purposes
     if (isReversed) {
      left = new EncoderFollower(leftTrajectory);
      right = new EncoderFollower(rightTrajectory);
    } else {
      left = new EncoderFollower(rightTrajectory);
      right = new EncoderFollower(leftTrajectory);
    }
  }


  /**
   * Creates a DriveMotionProfile from a text file
   * @param fileName path to trajectory file inside the output folder in the deploy directory
   */
  public DriveMotionProfile(String fileName, boolean isReversed) {
    try {
      this.isReversed = isReversed;
      SmartDashboard.putString("current-command", "DriveMotionProfile");

      String binary = "traj";
      String csv = "csv";
      boolean encoderFollowersSet = false;

      requires(Robot.driveBase);

      String leftPathname = Filesystem.getDeployDirectory() + "/output/" + fileName.replace(".pf1", ".left.pf1");
      String rightPathname = Filesystem.getDeployDirectory() + "/output/" + fileName.replace(".pf1", ".right.pf1");
      Logger.consoleLog("Left path name: " + leftPathname);
      Logger.consoleLog("Right path name: " + rightPathname);
      File leftFile = new File(leftPathname);
      File rightFile = new File(rightPathname);

      String extension = "";
      int i = fileName.lastIndexOf('.');
      if (i > 0) {
        extension = fileName.substring(i + 1);
      }

      if (extension.equals(csv)) {
        leftTrajectory = Pathfinder.readFromCSV(leftFile);
        rightTrajectory = Pathfinder.readFromCSV(rightFile);
      } else if (extension.equals(binary)) {
        leftTrajectory = Pathfinder.readFromFile(leftFile);
        rightTrajectory = Pathfinder.readFromFile(rightFile);
      } else {
        // // Set the max velocity to something lower than the actual max velocity,
        // otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn
        // properly
        // // For here let's set it to
        // config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
        // Trajectory.Config.SAMPLES_HIGH, TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION,
        // MAX_JERK);
        // Trajectory trajectory = Pathfinder.generate(waypoints, config);
        // // follow old method of splitting one trajectory into left and right if
        // generated with waypoints
        // modifier = new TankModifier(trajectory).modify(RobotConstants.WHEEL_BASE);
        // left = new EncoderFollower(modifier.getLeftTrajectory());
        // right= new EncoderFollower(modifier.getRightTrajectory());
        // encoderFollowersSet = true;
        throw new FileNotFoundException("DriveMotionProfile did not receive a valid filename.");
      }
      // Don't need this if reading in left and right trajectories
      // modifier = new TankModifier(trajectory).modify(RobotConstants.WHEEL_BASE);

      // TODO figure out why these are switched and switch them back
      if (isReversed) {
        left = new EncoderFollower(leftTrajectory);
        right = new EncoderFollower(rightTrajectory);
      } else {
        left = new EncoderFollower(rightTrajectory);
        right = new EncoderFollower(leftTrajectory);
      }
    } catch (FileNotFoundException ex) {
      try {
        DriverStation.reportError("Error reading in File for DriveMotionProfile" + ex.getMessage(), true);
        leftTrajectory = Pathfinder.readFromCSV(new File(Filesystem.getDeployDirectory() + "/output/DriveOffPlatform.left.pf1"));
        rightTrajectory = Pathfinder.readFromCSV(new File(Filesystem.getDeployDirectory() + "/output/DriveOffPlatform.right.pf1"));
      }
      catch(Exception e) {

      }
      // super.cancel();
      // return;
    }
    catch(IOException e) {

    }
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // Zeroes the NavX to avoid errors with pathing direction.
    Robot.sensors.zeroYaw();
    // For real matches instantiate this in constructor.
    // left = new EncoderFollower(modifier.getLeftTrajectory());
    // right= new EncoderFollower(modifier.getRightTrajectory());

    int initialLeftEncoder, initialRightEncoder;
    // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
    // 'getEncPosition' function.
    // 1024 is the amount of encoder ticks per full revolution
    // Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters

  //Default: use DIO Encoders

    if(isReversed) {
        initialLeftEncoder = -1 * Robot.driveBase.getLeftMustangDriveBaseEncoder().getPositionTicks();
        initialRightEncoder = -1 * Robot.driveBase.getRightMustangDriveBaseEncoder().getPositionTicks();
    }
    else {
        initialLeftEncoder = Robot.driveBase.getLeftMustangDriveBaseEncoder().getPositionTicks();
        initialRightEncoder = Robot.driveBase.getRightMustangDriveBaseEncoder().getPositionTicks();
    }

    if (Robot.driveBase.getLeftDIOEncoder() != null && Robot.driveBase.getRightDIOEncoder() != null) {
      left.configureEncoder(initialLeftEncoder, RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
      right.configureEncoder(initialRightEncoder, RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
    } else {
      left.configureEncoder(initialLeftEncoder, RobotConstants.SPARK_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
      right.configureEncoder(initialRightEncoder, RobotConstants.SPARK_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
    }

    Logger.consoleLog("isReversed: " + isReversed);
    Logger.consoleLog("initial encoders: " + initialLeftEncoder + ", " + initialRightEncoder);

    // Set up Robot for Auton Driving
    Robot.driveBase.initBrakeMode();
    Logger.consoleLog("PID values: " + P + ", " + I + ", " + D + ", " + (1.0/MAX_VELOCITY) + ", " + KA);

    // The first argument is the proportional gain. Usually this will be quite high
    // The second argument is the integral gain. This is unused for motion profiling
    // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
    left.configurePIDVA(P, I, D, (1.0 / MAX_VELOCITY), KA);
    right.configurePIDVA(P, I, D, (1.0 / MAX_VELOCITY), KA);

    // Logger.consoleLog("Initialized DriveMotionProfile: InitialLeftEncoder: %s, InitialRightEncoder: %s, InitialAngle: %s", initialLeftEncoder, initialRightEncoder, Robot.sensors.getYawDouble());

    executeCount = 0;
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    int leftEncoder, rightEncoder;

  //Default: use DIO Encoders

    if(isReversed) {
      leftEncoder = -1 * Robot.driveBase.getLeftMustangEncoderPositionInTicks();
      rightEncoder = -1 * Robot.driveBase.getRightMustangEncoderPositionInTicks();
    }
    else {
      leftEncoder = Robot.driveBase.getLeftMustangEncoderPositionInTicks();
      rightEncoder = Robot.driveBase.getRightMustangEncoderPositionInTicks();
    }  

    // System.out.println("encoders: " + leftEncoder + ", " + rightEncoder);
  
    // System.out.println("Right Encoder: " + rightEncoder + ", LeftEncoder: " + leftEncoder);
    double l = left.calculate(leftEncoder);
    double r = right.calculate(rightEncoder);

    /* Code below dependent on the NavX. Right now, is not required for this command. If uncommented, need a condition to 
      check if NavX is null, otherwise if NavX is unplugged, robot code will crash
     */

    double turn = 0;

     if(!Robot.sensors.isNavXNull()) {
      // Calculates the angle offset for PID
      // // It tracks the wrong angle (mirrors the correct one)
      double gyroHeading;
      if(isReversed) {
        gyroHeading = Pathfinder.boundHalfDegrees(-1 * Robot.sensors.getYawDoubleForPathfinder());   // Assuming the gyro is giving a value in degrees
      }
      else {
        gyroHeading = Pathfinder.boundHalfDegrees(Robot.sensors.getYawDoubleForPathfinder());   // Assuming the gyro is giving a value in degrees
      }
      double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));  // Should also be in degrees 
      // Make sure gyro and desired angle match up [-180, 180], navX reports the opposite orientation as Pathfinder expects
      double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);    
      
      // TODO MAKE THE -1 HERE MATCH uP WITH THE DIRECTION THE ROBOT SHOULD TURN
      turn = 0.8 * (-1.0/ANGLE_DIVIDE_CONSTANT) * angleDifference;
     }
    
    double leftOutput = l + turn;
    double rightOutput = r - turn;
    if (isReversed) {
      leftOutput *= -1;
      rightOutput *= -1;
    }

    Logger.consoleLog("encoders: " + leftEncoder + ", " + rightEncoder + " outputs: " + leftOutput + ", " + rightOutput);
    // Drives the bot based on the input
    Robot.driveBase.tankDrive(leftOutput, rightOutput, false); 

    // if(executeCount % 5 == 0) {
    //   Logger.consoleLog("Execute: gyroHeading: %s, desiredHeading: %s, angleDifference: %s, angleDivideConstant: %s, turn: %s, leftOuput: %s, rightOutput: %s", gyroHeading, desiredHeading, angleDifference, ANGLE_DIVIDE_CONSTANT, turn, leftOutput , rightOutput) ;
    // }

    executeCount++;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //This should probably be made to take into account robot speed as well in case the robot hits something and is unable to move further.
    return left.isFinished() && right.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveBase.stop();
    if(!Robot.sensors.isNavXNull())
       Logger.consoleLog("EndingAngle: %s, LeftTicksTraveled: %s, RightTicksTraveled: %s, DistanceTraveled: %s", Pathfinder.boundHalfDegrees(Robot.sensors.getYawDoubleForPathfinder()), 
                     (Robot.driveBase.getLeftMustangEncoderPositionInTicks() - initialLeftEncoder), (Robot.driveBase.getRightMustangEncoderPositionInTicks() - initialRightEncoder), DriveBase.convertDriveBaseTicksToInches(MathUtils.average((double)(Robot.driveBase.getLeftMustangEncoderPositionInTicks() - initialLeftEncoder), (double)(Robot.driveBase.getRightMustangEncoderPositionInTicks() - initialRightEncoder))));
    else
       Logger.consoleLog("LeftTicksTraveled: %s, RightTicksTraveled: %s, DistanceTraveled: %s", (Robot.driveBase.getLeftMustangEncoderPositionInTicks() - initialLeftEncoder), (Robot.driveBase.getRightMustangEncoderPositionInTicks() - initialRightEncoder), DriveBase.convertDriveBaseTicksToInches(MathUtils.average((double)(Robot.driveBase.getLeftMustangEncoderPositionInTicks() - initialLeftEncoder), (double)(Robot.driveBase.getRightMustangEncoderPositionInTicks() - initialRightEncoder))));
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog() ;
    end();
  }

}

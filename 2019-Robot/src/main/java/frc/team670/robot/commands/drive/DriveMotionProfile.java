
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.team670.robot.commands.drive;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
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
  private Trajectory trajectory;
  private Trajectory leftTrajectory, rightTrajectory;
  private TankModifier modifier;
  private EncoderFollower left, right;
  // Max Velocities in m/s. For generation, we need it to be lower than the actual max velocity, 
  // otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn properly.
  private static final String BASE_PATH_NAME = "home/deploy/";
  private static final double MAX_VELOCITY = 120, MAX_ACCELERATION = 60, MAX_JERK = 60; // Equivalent units in inches
  private static final double TIME_STEP = 0.05;
  private static final double WHEEL_BASE = 25; //Inches
  private static final double ANGLE_DIVIDE_CONSTANT = 240.0; // Default = 80


  private static final double P = 0.5, I = 0, D = 0, KA= 0;

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
    trajectory = Pathfinder.generate(waypoints, config);
    modifier = new TankModifier(trajectory).modify(WHEEL_BASE);

     // TODO In the future maybe do this in initialize so when the Command is rerun it starts over
    left = new EncoderFollower(modifier.getLeftTrajectory());
    right= new EncoderFollower(modifier.getRightTrajectory());
  }


  /**
   * Creates a DriveMotionProfile from a text file
   * @param fileName path to trajectory file inside the output folder in the deploy directory
   */
  public DriveMotionProfile(String fileName, boolean isReversed) {

    this.isReversed = isReversed;

    String binary = "traj";
    String csv = "csv";
    boolean encoderFollowersSet = false;
    
    requires(Robot.driveBase);

    String leftPathname = Filesystem.getDeployDirectory() + "/output/" + fileName.replace(".pf1", ".left.pf1");
    String rightPathname = Filesystem.getDeployDirectory() + "/output/" + fileName.replace(".pf1", ".right.pf1");
    System.out.println("Left path name: " + leftPathname);
    System.out.println("Right path name: " + rightPathname);
    File leftFile = new File(leftPathname);
    File rightFile = new File(rightPathname);

    String extension = "";
    int i = fileName.lastIndexOf('.');
    if (i > 0) {
        extension = fileName.substring(i+1);
    }


    if (extension.equals(csv)){
      leftTrajectory = Pathfinder.readFromCSV(leftFile);
      rightTrajectory = Pathfinder.readFromCSV(rightFile);
    }
    else if (extension.equals(binary)) {
      leftTrajectory = Pathfinder.readFromFile(leftFile);
      rightTrajectory = Pathfinder.readFromFile(rightFile);
    } else {
      // Set the max velocity to something lower than the actual max velocity, otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn properly
      // For here let's set it to 
      config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, TIME_STEP, MAX_VELOCITY, MAX_ACCELERATION, MAX_JERK);
      trajectory = Pathfinder.generate(waypoints, config);
      // follow old method of splitting one trajectory into left and right if generated with waypoints
      modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVEBASE_TRACK_WIDTH);
      left = new EncoderFollower(modifier.getLeftTrajectory());
      right= new EncoderFollower(modifier.getRightTrajectory());
      encoderFollowersSet = true;
    }
    modifier = new TankModifier(trajectory).modify(WHEEL_BASE);

    if (!encoderFollowersSet) { // avoids re setting if trajectory is generated from waypoints
      left = new EncoderFollower(leftTrajectory);
      right = new EncoderFollower(rightTrajectory);
      encoderFollowersSet = true;
    }

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // TODO make sure angle actually serts to zero (firmware update NavX)
    // TODO Think through what we want out of angle, maybe go off an initial angle
    Robot.sensors.zeroYaw();

    // For real matches instantiate this in constructor.
    left = new EncoderFollower(modifier.getLeftTrajectory());
    right= new EncoderFollower(modifier.getRightTrajectory());

    double initialLeftEncoder, initialRightEncoder;
    if(isReversed) {
      initialLeftEncoder = -1 * Robot.driveBase.getLeftDIOEncoderPosition();
      initialRightEncoder = -1 * Robot.driveBase.getRightDIOEncoderPosition();
    }
    else {
      initialLeftEncoder = Robot.driveBase.getLeftDIOEncoderPosition();
      initialRightEncoder = Robot.driveBase.getRightDIOEncoderPosition();
    }

    // Set up Robot for Auton Driving
    Robot.driveBase.initAutonDrive();

    // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
    // 'getEncPosition' function.
    // 1000 is the amount of encoder ticks per full revolution
    // Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters

    if(isReversed){
      left.configureEncoder(-1 * Robot.driveBase.getLeftDIOEncoderPosition(), RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
      right.configureEncoder(-1 * Robot.driveBase.getRightDIOEncoderPosition(), RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);  
    } else{
      left.configureEncoder(Robot.driveBase.getLeftDIOEncoderPosition(), RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
      right.configureEncoder(Robot.driveBase.getRightDIOEncoderPosition(), RobotConstants.DIO_TICKS_PER_ROTATION, RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);  
    }

    // The first argument is the proportional gain. Usually this will be quite high
    // The second argument is the integral gain. This is unused for motion profiling
    // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
    left.configurePIDVA(P, I, D, (1 / MAX_VELOCITY), KA);
    right.configurePIDVA(P, I, D, (1 / MAX_VELOCITY), KA);

    Logger.consoleLog("Initialized DriveMotionProfile: InitialLeftEncoder: %s, InitialRightEncoder: %s, InitialAngle: %s", initialLeftEncoder, initialRightEncoder, Robot.sensors.getYawDouble());

    executeCount = 0;
   
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    // Example code from Jaci's Motion profiling, calculates distance for PID
    /*
    * LEFT ENCODER IS BACKWARDS SO WE MULTIPLY IT'S VALUE BY -1 TO FLIP IT
    */

    int leftEncoder, rightEncoder;
    if(isReversed) {
      leftEncoder = -1 * Robot.driveBase.getLeftDIOEncoderPosition();
      rightEncoder = -1 * Robot.driveBase.getRightDIOEncoderPosition();
    }
    else {
      leftEncoder = Robot.driveBase.getLeftDIOEncoderPosition();
      rightEncoder = Robot.driveBase.getRightDIOEncoderPosition();
    }
  
    // System.out.println("Right Encoder: " + rightEncoder + ", LeftEncoder: " + leftEncoder);
    double l = left.calculate(leftEncoder);
    double r = right.calculate(rightEncoder);
    
    // Calculates the angle offset for PID
    // It tracks the wrong angle (mirrors the correct one)
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
    
    // Making this constant higher helps prevent the robot from overturning (overturning also will make it not drive far enough in the correct direction)
    // TODO MAKE THE -1 HERE MATCH uP WITH THE DIRECTION THE ROBOT SHOULD TURN
    double turn = 0.8 * (-1.0/ANGLE_DIVIDE_CONSTANT) * angleDifference;
    
    double leftOutput = l + turn;
    double rightOutput = r - turn;

    // Drives the bot based on the input
    Robot.driveBase.tankDrive(leftOutput, rightOutput, false); 

    if(executeCount % 5 == 0) {
      Logger.consoleLog("Execute: gyroHeading: %s, desiredHeading: %s, angleDifference: %s, angleDivideConstant: %s, turn: %s, leftOuput: %s, rightOutput: %s", gyroHeading, desiredHeading, angleDifference, ANGLE_DIVIDE_CONSTANT, turn, leftOutput , rightOutput) ;
    }

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
    Logger.consoleLog("Ended. EndingAngle: %s, LeftTicksTraveled: %s, RightTicksTraveled: %s, DistanceTraveled: %s", Pathfinder.boundHalfDegrees(Robot.sensors.getYawDoubleForPathfinder()), 
                     (Robot.driveBase.getLeftDIOEncoderPosition() - initialLeftEncoder), (Robot.driveBase.getRightDIOEncoderPosition() - initialRightEncoder), MathUtils.convertDriveBaseTicksToInches(MathUtils.average((double)(Robot.driveBase.getLeftDIOEncoderPosition() - initialLeftEncoder), (double)(Robot.driveBase.getRightDIOEncoderPosition() - initialRightEncoder))));
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted") ;
    end();
  }

}


/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/



package frc.team670.robot.commands.auto;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.utils.Logger;
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
  private TankModifier modifier;
  private EncoderFollower left, right;
  private final int TICKS_PER_ROTATION = 4096;
  // Max Velocities in m/s. For generation, we need it to be lower than the actual max velocity, 
  // otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn properly.
  private static final double real_maxVelocity = 0.8, generation_MaxVelocity = 0.8;
  private static final String BASE_PATH_NAME = "home/deploy/";
  // Values for logging purposes
  private final int executeLogInterval = 8;
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

    config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, (0.02), (generation_MaxVelocity), (2.0), 60.0);
    trajectory = Pathfinder.generate(waypoints, config);
    modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVEBASE_TRACK_WIDTH );

     // TODO In the future maybe do this in initialize so when the Command is rerun it starts over
    left = new EncoderFollower(modifier.getLeftTrajectory());
    right= new EncoderFollower(modifier.getRightTrajectory());

  }


  /**
   * Creates a DriveMotionProfile from a text file
   * @param fileName path to trajectory file inside the deploy folder
   */
  public DriveMotionProfile(String fileName, boolean isReversed) {

    this.isReversed = isReversed;

    String binary = "traj";
    String csv = "csv";
    
    requires(Robot.driveBase);

    String pathname = Filesystem.getDeployDirectory() + fileName;
    System.out.println("Path Name: " + pathname);
    File file = new File(pathname);

    String extension = "";
    int i = fileName.lastIndexOf('.');
    if (i > 0) {
        extension = fileName.substring(i+1);
    }


    if (extension.equals(csv)){
      System.out.println("File Name: " + file.getName());
      trajectory = Pathfinder.readFromCSV(file);
    }
    else if (extension.equals(binary))
      trajectory = Pathfinder.readFromFile(file);
    else {
      // Set the max velocity to something lower than the actual max velocity, otherwise we get motor outputs >1.0 and <-1.0 which makes us unable to turn properly
      // For here let's set it to 
      config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, (0.02), (generation_MaxVelocity), (2.0), 60.0);
      trajectory = Pathfinder.generate(waypoints, config);
    }

    modifier = new TankModifier(trajectory).modify(RobotConstants.DRIVEBASE_TRACK_WIDTH);
    left = new EncoderFollower(modifier.getLeftTrajectory());
    right= new EncoderFollower(modifier.getRightTrajectory());

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    // TODO make sure angle actually serts to zero (firmware update NavX)
    // TODO Think through what we want out of angle, maybe go off an initial angle
    Robot.sensors.resetNavX();

    initialLeftEncoder = Robot.driveBase.getLeftEncoderPosition();
    initialRightEncoder = Robot.driveBase.getRightEncoderPosition();

    // Set up Robot for Auton Driving
    Robot.driveBase.initAutonDrive();

    // Encoder Position is the current, cumulative position of your encoder. If you're using an SRX, this will be the
    // 'getEncPosition' function.
    // 1000 is the amount of encoder ticks per full revolution
    // Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters
    left.configureEncoder(Robot.driveBase.getLeftEncoderPosition(), TICKS_PER_ROTATION, RobotConstants.WHEEL_DIAMETER);
    right.configureEncoder(Robot.driveBase.getRightEncoderPosition(), TICKS_PER_ROTATION, RobotConstants.WHEEL_DIAMETER);

    // The first argument is the proportional gain. Usually this will be quite high
    // The second argument is the integral gain. This is unused for motion profiling
    // The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory
    // The fourth argument is the velocity ratio. This is 1 over the maximum velocity you provided in the 
    //      trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read)
    // The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker
    left.configurePIDVA(1.0, 0.0, 0.0, (1) / (generation_MaxVelocity), (0.0));
    right.configurePIDVA(1.0, 0.0, 0.0, (1) / (generation_MaxVelocity), (0.0));

    if(isReversed){
    double l = left.calculate(-Robot.driveBase.getLeftEncoderPosition());// Negate encoder count so robot thinks its going forward when going backwards
    double r = right.calculate(-Robot.driveBase.getRightEncoderPosition());

    double angleDifference = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()) + Robot.sensors.getYawDouble());
    double turn = 2 * (-1.0 / 80.0) * angleDifference; // We used 2 as it seemed to work better after trial and error
			
    Robot.driveBase.tankDrive(-(r - turn), -(l + turn));// Left Side Output, Right Side Output. We use the right output for the left side and the left side output for the right side, then negate everything
    }

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
    int leftEncoder = Robot.driveBase.getLeftEncoderPosition();
    int rightEncoder = Robot.driveBase.getRightEncoderPosition();
    // System.out.println("Right Encoder: " + rightEncoder + ", LeftEncoder: " + leftEncoder);
    double l = left.calculate(leftEncoder);
    double r = right.calculate(rightEncoder);
    
    // Calculates the angle offset for PID
    double gyroHeading = Pathfinder.boundHalfDegrees(Robot.sensors.getYawDoubleForPathfinder());   // Assuming the gyro is giving a value in degrees
    double desiredHeading = Pathfinder.boundHalfDegrees(Pathfinder.r2d(left.getHeading()));  // Should also be in degrees 
    // Make sure gyro and desired angle match up [-180, 180], navX reports the opposite orientation as Pathfinder expects
    double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);    
    
    // Making this constant higher helps prevent the robot from overturning (overturning also will make it not drive far enough in the correct direction)
    double angleDivideConstant = 240.0; // Default = 80
    // TODO MAKE THE -1 HERE MATCH uP WITH THE DIRECTION THE ROBOT SHOULD TURN
    double turn = 0.8 * (-1.0/angleDivideConstant) * angleDifference;
    
    double leftOutput = l + turn;
    double rightOutput = r - turn;

    // Drives the bot based on the input
    Robot.driveBase.tankDrive(leftOutput, rightOutput); 

    if(executeCount % 5 == 0) {
      Logger.consoleLog("Execute: gyroHeading: %s, desiredHeading: %s, angleDifference: %s, angleDivideConstant: %s, turn: %s, leftOuput: %s, rightOutput: %s", gyroHeading, desiredHeading, angleDifference, angleDivideConstant, turn, leftOutput , rightOutput) ;
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
    Robot.driveBase.tankDrive(0, 0);
    Logger.consoleLog("Ended. EndingAngle: %s, EndingLeftTicks: %s, EndingRightTicks: %s", Pathfinder.boundHalfDegrees(Robot.sensors.getYawDoubleForPathfinder()), 
                     (Robot.driveBase.getLeftEncoderPosition() - initialLeftEncoder), (Robot.driveBase.getRightEncoderPosition() - initialRightEncoder));
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Logger.consoleLog("Interrupted") ;
    end();
  }

}

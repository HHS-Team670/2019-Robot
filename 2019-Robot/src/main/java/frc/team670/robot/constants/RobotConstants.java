package frc.team670.robot.constants;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 32, ROBOT_WIDTH = 27.5, DRIVEBASE_TO_GROUND = 2.162;

    // Drive Base Dimensions TODO set these
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = (1/DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;

    public static final double DRIVEBASE_METERS_PER_ROTATION = (1/DRIVEBASE_GEAR_RATIO) * DRIVE_BASE_WHEEL_DIAMETER * Math.PI * 0.0254;

    /** The number of ticks per rotation of a drivebase wheel for the DIO Encoders  */
    public static final int DIO_TICKS_PER_ROTATION = 1024;
    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));
     /** The number of ticks per rotation of a drivebase wheel for the SPARK Encoders  */
     public static final int SPARK_TICKS_PER_ROTATION = 1024;

    // LEDs
    public static final int LED_PORT = 5801;

    // Vision Constants
    public static final double VISION_ERROR_CODE = -99999;

    // Pathfinder Constants (DriveMotionProfile)
    public static final double KU = 0.275; //Original P value
    public static final double TU = 4.46; //Oscillation in seconds
    public static final double PROPORTION = 0.2*KU;
    public static final double INTEGRAL = 0;//0.05*KU/TU;
    public static final double DERIVATIVE = 5*KU*TU/40;
    public static final double WHEEL_BASE = 25.662; //measured 1/26/19

    //Elbow constants
    public static final int PEAK_AMPS = 0; // check the peak limit and set again
    public static final int TIMEOUT_MS = 0; //  
    public static final int PEAK_TIME_MS = 0; //  Duration after current exceed peak current to trigger current limit

    // Climbing Current Limits
    public static final int CLIMB_CURRENT_LIMIT = 10; // TODO figure required limited current
    public static final int NORMAL_CURRENT_LIMIT = 0; // TODO figure out normal current

    //Extension constants
    public static final int EXTENSION_ENCODER_OUT = 0;
    public static final double DEFAULT_EXTENSION_POWER = 0; 
    public static final double INCHES_TO_TRAVEL_ONTO_PLATFORM = 4;

    //Arm PID Constants
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 0;
    public static final int ALLOWABLE_PID_ERROR = 0;

    // Different values/encoder positions to set arm to
    public static int ARM_RESET_TIMEOUTMS = 0;

    public static final double ksVolts = 0.11; //0.224;
    public static final double kvVoltSecondsPerMeter = 2.27; //2.22;
    public static final double kaVoltSecondsSquaredPerMeter = 0.765; // 0.715;

    // "WHEEL_BASE" is really track width
    public static final double kTrackwidthInches = WHEEL_BASE;
    public static final double kTrackwidthMeters = WHEEL_BASE * 0.0254;

    // Example value only - as above, this must be tuned for your drive!
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
    public static final double kPDriveVel = 4;
    public static final double kIDriveVel = 0.00;
    public static final double kDDriveVel = 0.0;
    public static final double kMaxSpeedInchesPerSecond = 12;
    public static final double kMaxAccelerationInchesPerSecondSquared = 12;

    public static final double kMaxSpeedMetersPerSecond = 4;//1; //0.305;
    public static final double kMaxAccelerationMetersPerSecondSquared = 4;//1; //0.305;

    public static final DifferentialDriveKinematicsConstraint kAutoPathConstraints =
        new DifferentialDriveKinematicsConstraint(kDriveKinematics,
                                                  kMaxSpeedMetersPerSecond);

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = .7;
    public static final boolean kNavXReversed = true;

}
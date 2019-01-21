package frc.team670.robot.constants;

import frc.team254.lib.util.control.Lookahead;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0;

    //Drive Base Dimensions TODO set these
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1/DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;
    /** The number of ticks per rotation of a drivebase wheel for the DIO Encoders  */
    public static final int DIO_TICKS_PER_ROTATION = 4096;

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
    public static final double DRIVEBASE_TRACK_WIDTH = 25; //TODO set this value to the actual

    // Pure Pursuit Constants
    public static final double MIN_LOOKAHEAD = 12.0; //Inches. TODO: set the value; 1 ft for now idk
    public static final double MAX_LOOKAHEAD = 18.0; //Inches. TODO: set this to something legit
    public static final double PATH_FOLLOWING_MAX_ACCEL = 0; //TODO: set value of this
    public static final double SEGMENT_COMPLETION_TOLERANCE = 0; //TODO: set value of this
    public static final double KTRACK_WIDTH_INCHES = 0; //TODO: figure out what this is
    public static final double KTRACK_SCRUB_FACTOR = 0; //TODO: figure out what this is
    public static final double MIN_LOOKAHEAD_SPEED = 0; //TODO: figure this out
    public static final double MAX_LOOKAHEAD_SPEED = 0; //TODO: figure this out
    public static final Lookahead LOOKAHEAD = new Lookahead(MIN_LOOKAHEAD, MAX_LOOKAHEAD,
    MIN_LOOKAHEAD_SPEED, MAX_LOOKAHEAD_SPEED);

    // Arm Constants
    public static final double ARM_HEIGHT = 0;
    public static final double CLAW_RADIUS = 0;
    public static final int FIXED_ARM_LENGTH = 0;

    //Elbow constants
    public static final int PEAK_AMPS = 0; // check the peak limit and set again
    public static final int TIMEOUT_MS = 0; //  
    public static final int PEAK_TIME_MS = 0; //  Duration after current exceed peak current to trigger current limit
    public static final int CLIMB_CURRENT_LIMIT = 10; // TODO figure required limited current
    public static final int NORMAL_CURRENT_LIMIT = 0; // TODO figure out normal current

    //Extension constants
    public static final int EXTENSION_ENCODER_OUT = 0;
    public static final double DEFAULT_EXTENSION_POWER = 0; 
    public static final double INCHES_TO_TRAVEL_ONTO_PLATFORM = 4;

    //Arm PID Constants
	public static final int kPIDLoopIdx = 0;
	public static final int kTimeoutMs = 10;
}
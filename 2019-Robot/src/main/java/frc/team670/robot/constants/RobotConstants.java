package frc.team670.robot.constants;

import frc.team254.lib.util.control.Lookahead;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0, DRIVEBASE_TO_GROUND = 0;

    // Drive Base Dimensions TODO set these
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1 / DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER
            * Math.PI;
    /**
     * The number of ticks per rotation of a drivebase wheel for the DIO Encoders
     */
    public static final int DIO_TICKS_PER_ROTATION = 4096;

    // TODO figure out real values for each constants for arm
    public static double EXTENSION_TICKS_PER_INCH = 2000; //Subject to change, measure real value
    public static double ELBOW_TICKS_PER_ROTATION = 2000; //Subject to change, check sources
    public static double WRIST_TICKS_PER_ROTATION = 2000; //Subject to change, check sources

    // LEDs
    public static final int LED_PORT = 5801;

    // Vision Constants
    public static final double VISION_ERROR_CODE = -99999;

    // Pathfinder Constants (DriveMotionProfile)
    public static final double KU = 0.275; // Original P value
    public static final double TU = 4.46; // Oscillation in seconds
    public static final double PROPORTION = 0.2 * KU;
    public static final double INTEGRAL = 0;// 0.05*KU/TU;
    public static final double DERIVATIVE = 5 * KU * TU / 40;
    public static final double DRIVEBASE_TRACK_WIDTH = 25; // TODO set this value to the actual

    // Pure Pursuit Constants
    public static final double MIN_LOOKAHEAD = 12.0; // Inches. TODO: set the value; 1 ft for now idk
    public static final double MAX_LOOKAHEAD = 18.0; // Inches. TODO: set this to something legit
    public static final double PATH_FOLLOWING_MAX_ACCEL = 0; // TODO: set value of this
    public static final double SEGMENT_COMPLETION_TOLERANCE = 0; // TODO: set value of this
    public static final double KTRACK_WIDTH_INCHES = 0; // TODO: figure out what this is
    public static final double KTRACK_SCRUB_FACTOR = 0; // TODO: figure out what this is
    public static final double MIN_LOOKAHEAD_SPEED = 0; // TODO: figure this out
    public static final double MAX_LOOKAHEAD_SPEED = 0; // TODO: figure this out
    public static final Lookahead LOOKAHEAD = new Lookahead(MIN_LOOKAHEAD, MAX_LOOKAHEAD, MIN_LOOKAHEAD_SPEED,
            MAX_LOOKAHEAD_SPEED);

    // Arm Constants
    public static final double ARM_HEIGHT_IN_INCHES = 0;
    public static final double CLAW_RADIUS_IN_INCHES = 0;
    public static final int FIXED_ARM_LENGTH_IN_INCHES = 0;


    public static int MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
    public static int MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

    // Different values/encoder positions to set arm to
    public static int ARM_RESET_TIMEOUTMS = 30;
    public static double ELBOW_START_POS = 6000;
    public static double ELBOW_FULL_FORWARD_POS = 12000;
    public static double ELBOW_FULL_BACKWARD_POS = -12000;

    public static double WRIST_START_POS = 6000;
    public static double WRIST_FULL_FORWARD_POS = 12000;
    public static double WRIST_FULL_BACKWARD_POS = -12000;

    public static double EXTENSION_IN_POS = 0;
    public static double EXTENSION_OUT_POS = 12000;

    // Climber Constants
    public static final int PISTON_ENCODER_FLAT = 0; // TODO set these
    public static final int PISTON_ENCODER_LEVEL_TWO = 0;
    public static final int PISTON_ENCODER_LEVEL_THREE = 0;

    // PID Constants
    public static final int kTimeoutMs = 0;
}
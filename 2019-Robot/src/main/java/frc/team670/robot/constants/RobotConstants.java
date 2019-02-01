package frc.team670.robot.constants;

import frc.team254.lib.util.control.Lookahead;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 32, ROBOT_WIDTH = 27.5, DRIVEBASE_TO_GROUND = 0;

    // Drive Base Dimensions TODO set these
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which
                                                            // one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1/DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;
    /** The number of ticks per rotation of a drivebase wheel for the DIO Encoders  */
    public static final int DIO_TICKS_PER_ROTATION = 1024;
    /** The number of ticks per inch of wheel travel */
    public static final int DIO_TICKS_PER_INCH = (int) (DIO_TICKS_PER_ROTATION / (Math.PI * DRIVE_BASE_WHEEL_DIAMETER));
     /** The number of ticks per rotation of a drivebase wheel for the SPARK Encoders  */
     public static final int SPARK_TICKS_PER_ROTATION = 1024;

    // TODO Set all of these!!!!
    public static double EXTENSION_TICKS_PER_MOTOR_ROTATION = 4096; //Subject to change, measure real value
    public static double EXTENSION_MOTOR_ROTATIONS_PER_INCH = 0.05371428571; //Should be real value
    /** Elbow will have an absolute Mag Encoder */
    public static double ELBOW_TICKS_PER_ROTATION = 4096; // Still needs to be set
    /** Wrist has an abolute Mag Encoder */
    public static double WRIST_TICKS_PER_ROTATION = 4096; //Subject to change, check sources

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
    
    // Arm Constants
    // TODO Set these
    public static final double ARM_HEIGHT_IN_INCHES = 5;
    public static final double CLAW_LENGTH_IN_INCHES = 8;
    public static final int FIXED_ARM_LENGTH_IN_INCHES = 0;
    public static final double OPERATOR_ARM_CONTROL_SCALAR = 0.5;

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

    public static int MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
    public static int MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

    // Different values/encoder positions to set arm to
    public static int ARM_RESET_TIMEOUTMS = 0;
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

    //Intake constants
    /** Intake will have an absolute Mag Encoder */
    public static double INTAKE_TICKS_PER_ROTATION = 4096; // Still needs to be set
}
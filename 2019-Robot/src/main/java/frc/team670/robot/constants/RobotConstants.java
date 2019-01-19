package frc.team670.robot.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import frc.team254.lib.util.control.*;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0;

    //Drive Base Dimensions TODO set these
    public static final double DRIVEBASE_GEAR_RATIO = 8.45; // 8.45 if low gear, 10.71 if high gear. TODO check which one it is
    /** Drive Base Wheel Diameter in Inches */
    public static final double DRIVE_BASE_WHEEL_DIAMETER = 6;
    /** Inches per rotation of the NEO motors on the drivebase */
    public static final double DRIVEBASE_INCHES_PER_ROTATION = 1/DRIVEBASE_GEAR_RATIO * DRIVE_BASE_WHEEL_DIAMETER * Math.PI;
    private static final double DIO_TICKS_PER_MOTOR_ROTATION = 300; // TODO Set this based on encoder
    /** The number of ticks per rotation of a drivebase wheel for the DIO Encoders  */
    public static final double DIO_TICKS_PER_ROTATION = 4690;

    // Vision Constants
    public static final double VISION_ERROR_CODE = -99999;

    // Pathfinder Constants (DriveMotionProfile)
    public static final double KU = 0.275; //Original P value
    public static final double TU = 4.46; //Oscillation in seconds
    public static final double PROPORTION = 0.2*KU;
    public static final double INTEGRAL = 0;//0.05*KU/TU;
    public static final double DERIVATIVE = 5*KU*TU/40;
    public static final double DRIVEBASE_TRACK_WIDTH = 25; //TODO set this value to the actual

    // LED Constants
    public static final int LED_PORT = 5810;

    // Pure Pursuit Constants
    public static final double minLookahead = 12.0; //Inches. TODO: set the value; 1 ft for now idk
    public static final double maxLookahead = 18.0; //Inches. TODO: set this to something legit
    public static final double pathFollowingMaxAccel = 0; //TODO: set value of this
    public static final double segmentCompletionTolerance = 0; //TODO: set value of this
    public static final double kTrackWidthInches = 0; //TODO: figure out what this is
    public static final double kTrackScrubFactor = 0; //TODO: figure out what this is
    public static final double minLookaheadSpeed = 0; //TODO: figure this out
    public static final double maxLookaheadSpeed = 0; //TODO: figure this out
    public static final Lookahead lookahead = new Lookahead(minLookahead, maxLookahead,
    minLookaheadSpeed, maxLookaheadSpeed);

    // Arm Constants
    public static final double ARM_BASE_HEIGHT = 0;
    public static final double CLAW_RADIUS = 0;
    public static final int PEAK_AMPS = 15; // check the peak limit and set again
    public static final int TIMEOUT_MS = 30; //  
    public static final int PEAK_TIME_MS = 0; //  Duration after current exceed peak current to trigger current limit
    public static final int TRIGGER_AMPS = 10; // TODO figure required limited current
}
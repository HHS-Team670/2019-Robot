package frc.team670.robot.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import frc.team254.lib.util.control.*;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0;

    //TODO set wheel diameter and ticks per rotation.
    public static final double WHEEL_DIAMETER_INCHES = 5, TICKS_PER_ROTATION = 4690;

    // Vision Constants
    public static final double VISION_ERROR_CODE = -99999;

    // Pathfinder Constants (DriveMotionProfile)
    public static final double KU = 0.275; //Original P value
    public static final double TU = 4.46; //Oscillation in seconds
    public static final double PROPORTION = 0.2*KU;
    public static final double INTEGRAL = 0;//0.05*KU/TU;
    public static final double DERIVATIVE = 5*KU*TU/40;
    public static final double DRIVEBASE_TRACK_WIDTH = 0; //TODO set this value
    public static final double WHEEL_DIAMETER = 6;

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

    // Pure Pursuit Path Following Constants TODO set these and make them final.
    public static double kMinLookAhead, 
    kMaxLookAhead,
    kMinLookAheadSpeed, kMaxLookAheadSpeed,
    kInertiaSteeringGain, kPathFollowingProfileKp,
    kPathFollowingProfileKi, kPathFollowingProfileKv,
    kPathFollowingProfileKffv, kPathFollowingProfileKffa,
    kPathFollowingMaxVel, kPathFollowingMaxAccel,
    kPathFollowingGoalPosTolerance, kPathFollowingGoalVelTolerance,
    kPathStopSteeringDistance;

}
package frc.team670.robot.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;
import frc.team254.lib.util.control.*;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0;

    // Pathfinder Required Values
    private static final double PATHFINDER_DT = 0.02, GENERATION_MAX_VELOCITY = 1.3, MAX_VELOCITY = 1.7, MAX_ACCELERATION = 2.0, 
                         MAX_JERK = 60. ;
    private static final int PATHFINDER_SAMPLES = Trajectory.Config.SAMPLES_HIGH;
    private static final FitMethod PATHFINDER_FIT_METHOD = Trajectory.FitMethod.HERMITE_CUBIC;
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

}
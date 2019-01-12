package frc.team670.robot.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.FitMethod;

public class RobotConstants {

    // Robot Dimensions
    public static final double ROBOT_LENGTH = 0, ROBOT_WIDTH = 0;

    //TODO set wheel diameter and ticks per rotation.
    public static final double WHEEL_DIAMETER_INCHES = 5, TICKS_PER_ROTATION = 4690;

    // Pathfinder Required Values
    private static final double PATHFINDER_DT = 0.02, GENERATION_MAX_VELOCITY = 1.3, MAX_VELOCITY = 1.7, MAX_ACCELERATION = 2.0, 
                         MAX_JERK = 60. ;
    private static final int PATHFINDER_SAMPLES = Trajectory.Config.SAMPLES_HIGH;
    private static final FitMethod PATHFINDER_FIT_METHOD = Trajectory.FitMethod.HERMITE_CUBIC;
    public static final double pathFollowingMaxAccel = 0; //TODO: set value of this
    public static final double segmentCompletionTolerance = 0; //TODO: set value of this

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

}
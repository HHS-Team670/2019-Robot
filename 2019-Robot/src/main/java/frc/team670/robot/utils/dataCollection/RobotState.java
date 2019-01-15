package frc.team670.robot.utils.dataCollection;
import frc.team254.lib.util.*;
import frc.team254.lib.util.math.RigidTransform2d;
import frc.team254.lib.util.math.Rotation2d;
import frc.team254.lib.util.math.Translation2d;
import frc.team254.lib.util.math.Twist2d;
import java.util.Map;


public class RobotState{
    private static RobotState instance_ = new RobotState();

    public static RobotState getInstance() {
        return instance_;
    }

    /**
     * TODO: return the robot's position on the field at a certain time. Linearly interpolates between stored robot positions
     * to fill in the gaps.
     */
    public synchronized RigidTransform2d getFieldToVehicle(double timestamp) {
        //stuff goes here
        return null;
    }

    /**
     * TODO: return the robot's latest position on the field. There should be a list of positions that are calculated
     * and this method should return the last entry on that list
     */
    public synchronized Map.Entry<InterpolatingDouble, RigidTransform2d> getLatestFieldToVehicle() {
        //stuff goes here 
        return null;
    }

}
package frc.team670.robot.utils;

import java.util.ArrayList;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

/**
 * Receives a stream of Double Arrays over NetworkTables containing calculated Segments and creates a Trajectory out of them.
 * @author shaylandias
 */
public class TrajectoryBuilder {

    /*
     * In the future, this can be set up to generate an ArrayList of Trajectories and add to it as it gets values. In this way
     * it can start driving almost instantly, and as long as more Trajectories are created faster than the robot drives, the
     * robot will not catch up and come to a stop. It would chain a bunch of DriveMotionProfile Commands together with the Trajectories.
     */

    private static final String DEFAULT_TABLE_NAME = "raspberryPi";
    private static final String DEFAULT_ENTRY_KEY = "segmentStream";
    private static final String DEFAULT_CONFIRMATION_ENTRY_KEY = "segmentStreamFinished";

    private boolean finishedBuilding;
    private Trajectory trajectory;

    private int confirmationListenerId, segmentListenerId;

    private ArrayList<Segment> segments;


    public TrajectoryBuilder() {
        this(DEFAULT_TABLE_NAME, DEFAULT_ENTRY_KEY, DEFAULT_CONFIRMATION_ENTRY_KEY);
    }

    /**
     * @param tableName Name of the table to get the entry from
     * @param entryKey The key for the NetworkTableEntry where the Segments will be streamed to as Double Arrays of 8 values in order: dt, x, y, position, velocity, acceleration, jerk, heading.
     * @param confirmationEntryKey The key for the NetworkTableEntry where the boolean of whether or not the Segments have been fully streamed will be stored.
     */
    public TrajectoryBuilder(String tableName, String entryKey, String confirmationEntryKey) {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();
        NetworkTable table = instance.getTable(tableName);
        trajectory = null;
        finishedBuilding = false;
        segmentListenerId = table.addEntryListener(entryKey, (table2, key2, entry, value, flags) -> {
            double[] vals = entry.getDoubleArray(new double[8]);
            segments.add(new Segment(vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6], vals[7]));
            Logger.consoleLog("Segment Received: %s", vals);
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

        confirmationListenerId = table.addEntryListener(confirmationEntryKey, (table2, key2, entry, value, flags) -> {
            finishedBuilding = entry.getBoolean(false);
            if(finishedBuilding){
                trajectory = new Trajectory(segments.toArray(new Segment[segments.size()]));
                table.removeEntryListener(confirmationListenerId);
                table.removeEntryListener(segmentListenerId);
                Logger.consoleLog("Finished Streaming and Building Trajectory.");
            }
        }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);

    }

    /**
     * @return True if finished building
     */
    public boolean isFinishedBuilding() {
        return finishedBuilding;
    }

    /**
     * Gets the Trajectory built 
     */
    public Trajectory getTrajectory() {
        return trajectory;
    }

}
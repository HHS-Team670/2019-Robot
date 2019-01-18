package frc.team670.robot.dataCollection;

import java.util.HashMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.constants.RobotConstants;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them Listeners to update the stored values
 * as they are changed.
 */
public class MustangPi {

    private HashMap<String, NetworkTableObject> entries;

    private VisionValues wallTarget; 

    // The keys for the NetworkTable entries that the raspberry pi is putting up. Ensure that these are placed on the raspi also. Maybe make a shared config file
    private static final String[] raspiKeys = new String[] {"reflect_tape_vision_data"};
    // The name of the subtable set on the raspberry pi
    private static final String tableName = "raspberryPi";

    // TODO Literally all of this code below needs a review from someone who knows what they're talking about - Shaylan

    public MustangPi() {
        this(raspiKeys);
    }

    private MustangPi(String[] keys) {
        entries = new HashMap<String, NetworkTableObject>();
        for(String key : keys){
           entries.put(key, new NetworkTableObject(key));
        }
        wallTarget = new VisionValues(raspiKeys[0]);
    }

    private class NetworkTableObject {

        private NetworkTableEntry entry;
        private String key;

        /**
         * The key of the NetworkTableEntry that this Object will be attached to.
         */
        public NetworkTableObject(String key) {
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            NetworkTable table = instance.getTable(tableName);
            entry = instance.getEntry(key);
            table.addEntryListener(key, (table2, key2, entry, value, flags) -> {
                this.entry = entry;
            }, EntryListenerFlags.kNew | EntryListenerFlags.kUpdate);
            this.key = key;
        }

        /**
         * Gets the value of the entry, this will be returned as a Double Array. Make sure to have a try/catch block for the Exception
         * @return A Double Array with the value or values last received from the pi. Fills with Vision Error Code if it can't get it
         */
        public double[] getValue() {
            if(entry.getType().equals(NetworkTableType.kDouble)) {
                return new double[] {entry.getDouble(RobotConstants.VISION_ERROR_CODE), RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE};
            } else if(entry.getType().equals(NetworkTableType.kDoubleArray)) {
                return entry.getDoubleArray(new double[]{RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE});
            } else {
                return new double[] {RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE, RobotConstants.VISION_ERROR_CODE};
            }
        }

        public NetworkTableEntry getEntry() {
            return entry;
        }

    }

    /** 
     * Angle to the vision target in degrees, as a PIDSource. Provides the VISION_ERROR_CODE if no value found.
     */
    public VisionValue_PIDSource getAngleToWallTarget() {
        return wallTarget.getAngle_PIDSource();
    }

    /**
     * Distance to the vision target in inches, as a PIDSource. Provides the VISION_ERROR_CODE if no value found.
     */
    public VisionValue_PIDSource getDistanceToWallTarget() {
        return wallTarget.getDistance_PIDSource();
    }

    public double[] getVisionValues() {
        if(entries == null) {
            System.out.println("entries null");
            return null;
        }
        return entries.get(raspiKeys[0]).getValue();
    }

    /**
     * Represents a set of vision data received from the raspberry pi containing an array of doubles in the form [angle, distance, timestamp]
     */
    public class VisionValues {
        private static final int ANGLE_INDEX = 0, DISTANCE_INDEX = 1, TIMESTAMP_INDEX = 2;
        private VisionValue_PIDSource angle, distance;
        
        private VisionValues(String keyName) {
            angle = new VisionValue_PIDSource(keyName, ANGLE_INDEX);
            distance = new VisionValue_PIDSource(keyName, DISTANCE_INDEX);
        }

        public VisionValue_PIDSource getAngle_PIDSource() {
            return angle;
        }

        public VisionValue_PIDSource getDistance_PIDSource() {
            return distance;
        }

        /**
         * Angle to the located target in degrees.
         */
        public double getAngle() {
            return angle.pidGet();
        }

        /**
         * Distance to the located target in inches.
         */
        public double getDistance() {
            return distance.pidGet();
        }

        /**
         * Gets the time stamp of the last vision calculation off the pi.
         */
        public double getTimeStamp() {
            return angle.getEntry(TIMESTAMP_INDEX);
        }

        /**
         * Returns true if a vision target is able to be located in the raspberry pi camera
         */
        public boolean canSeeVisionTarget() {
            return !MathUtils.doublesEqual(angle.pidGet(), RobotConstants.VISION_ERROR_CODE);
        }
    }

    /**
     * Implements a VisionValue (distance or angle) as a PIDSource
     */
    public class VisionValue_PIDSource implements PIDSource {

        private PIDSourceType pidSourceType;
        private String key;
        private int indexOfValue;

        private VisionValue_PIDSource(String keyName, int indexOfValue) {
            pidSourceType = PIDSourceType.kDisplacement;
            key = keyName;
            this.indexOfValue = indexOfValue;
        }

        @Override
        public double pidGet() {
            return getEntry(indexOfValue);
        }

        @Override
        public PIDSourceType getPIDSourceType() {
            return pidSourceType;
        }

        @Override
        public void setPIDSourceType(PIDSourceType pidSource) {
            pidSourceType = pidSource;
        }

        /**
         * Gets the entry at the given index in the array at the NetworkTable entry that corresponds to this object's key.
         */
        public double getEntry(int index) {
            if (entries.get(key)==null) {
                return RobotConstants.VISION_ERROR_CODE; // If no data found, returns VISION_ERROR_CODE
            }
            double result = entries.get(key).getValue()[index];
            return result;
        }

    }

}
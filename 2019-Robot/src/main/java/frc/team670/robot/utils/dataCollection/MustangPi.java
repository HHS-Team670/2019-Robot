package frc.team670.robot.utils.dataCollection;

import java.util.HashMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them
 * Listeners to update the stored values when they are changed
 */
public class MustangPi {

    private HashMap<String, NetworkTableObject> entries;

    // The keys for the NetworkTable entries that the raspberry pi is putting up
    private static final String[] raspiKeys = new String[] {};
    // The name of the subtable set on the raspberry pi
    private static final String tableName = "raspberryPi";

    // TODO Literally all of this code below needs a review from someone who knows what they're talking about - Shaylan

    public MustangPi() {
        this(raspiKeys);
    }

    private MustangPi(String[] keys) {
        for(String key : keys){
            entries.put(key, new NetworkTableObject(key));
        }
    }

    public static class NetworkTableObject {

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
         * @return A Double Array with the value or values last received from the pi.
         * @exception IllegalStateException Throws if the value received from the pi was Unassigned or not a Double or Double Array
         */
        public double[] getValue() throws IllegalStateException {
            if(entry.getType().equals(NetworkTableType.kDouble)) {
                return new double[] {entry.getDouble(0)};
            } else if(entry.getType().equals(NetworkTableType.kDoubleArray)) {
                return entry.getDoubleArray(new double[]{});
            } else {
                throw new IllegalStateException("Entry was not a Double, Double Array, or Unassigned");
            }
        }

        public NetworkTableEntry getEntry() {
            return entry;
        }

    }


}
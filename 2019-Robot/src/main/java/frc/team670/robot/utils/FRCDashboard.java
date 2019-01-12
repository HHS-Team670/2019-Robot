package frc.team670.robot.utils;

import java.util.ArrayList;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;


/**
 * A wrapper class for FRCDashboard
 */
public class FRCDashboard {

    private NetworkTable table;
    private NetworkTableInstance instance;

    private ArrayList<NetworkTableEntry> entries;
    private NetworkTableEntry autonChooser;

    /**
     * Constucts an object connecting to the given networktables table
     * 
     * @param tableName the name of the table to connect to
     */
    public FRCDashboard(String tableName) {
        instance = NetworkTableInstance.getDefault();
        table = instance.getTable(tableName);
        instance.startClientTeam(670);
        instance.startDSClient();

        entries = new ArrayList<NetworkTableEntry>();
        autonChooser = table.getEntry("auton-chooser");
        entries.add(autonChooser);
    }

    public void putString(String key, String value) {
        for (NetworkTableEntry entry : entries) {
            if (entry.getName().equals(key)) {
                entry.setString(value);
            }
        }
    }

    public void putDouble(String key, double value) {
        for (NetworkTableEntry entry : entries) {
            if (entry.getName().equals(key)) {
                entry.setDouble(value);
            }
        }
    }

    public double getDouble(String key) {
        NetworkTableValue received = getValue(key);
        return received.getDouble();
    }

    public String getString(String key) {
        NetworkTableValue received = getValue(key);
        return received.getString();
    }

    private NetworkTableValue getValue(String key) {
        return table.getEntry(key).getValue();
    }

}
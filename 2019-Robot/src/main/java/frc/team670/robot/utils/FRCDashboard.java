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

    
    private static NetworkTableInstance instance = NetworkTableInstance.getDefault();
    private static NetworkTable table = instance.getTable("SmartDashboard");

    private static ArrayList<NetworkTableEntry> entries = new ArrayList<NetworkTableEntry>();
    private NetworkTableEntry autonChooser;

    /**
     * Constucts an object connecting to the given networktables table
     * 
     * @param tableName the name of the table to connect to
     */
    private FRCDashboard() {
        instance.startClientTeam(670);
        instance.startDSClient();

        // SAMPLE USE CASE
        // autonChooser = table.getEntry("auton-chooser");
        // entries.add(autonChooser);
    }

    public static void putString(String key, String value) {
        for (NetworkTableEntry entry : entries) {
            if (entry.getName().equals(key)) {
                entry.setString(value);
            }
        }
    }

    public static void putDouble(String key, double value) {
        for (NetworkTableEntry entry : entries) {
            if (entry.getName().equals(key)) {
                entry.setDouble(value);
            }
        }
    }

    public static double getDouble(String key) {
        NetworkTableValue received = getValue(key);
        return received.getDouble();
    }

    public static String getString(String key) {
        NetworkTableValue received = getValue(key);
        return received.getString();
    }

    private static NetworkTableValue getValue(String key) {
        return table.getEntry(key).getValue();
    }

}
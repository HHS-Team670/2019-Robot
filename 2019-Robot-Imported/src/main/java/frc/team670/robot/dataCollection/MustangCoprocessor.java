package frc.team670.robot.dataCollection;

import java.util.HashMap;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Stores values off of NetworkTables for easy retrieval and gives them Listeners to update the stored values
 * as they are changed.
 */
public class MustangCoprocessor {

    private HashMap<String, NetworkTableObject> entries;

    private Solenoid backLedRing;
    private VisionValues wallTarget; 

    // The keys for the NetworkTable entries that the raspberry pi is putting up. Ensure that these are placed on the raspi also. Maybe make a shared config file
    private static final String[] NETWORK_TABLE_KEYS = new String[] {"reflect_tape_vision_data"};
    // The name of the subtable set on the raspberry pi
    private static final String TABLE_NAME = "SmartDashboard";

    //Vision Constants
    public static final double HIGH_TARGET_HEIGHT = 39.125; //inches
    public static final double LOW_TARGET_HEIGHT = 31.5; //inches

    private static final double BACK_CAMERA_HORIZONTAL_OFFSET = 8; //inches
    private static final double BACK_CAMERA_HEIGHT = 12.25;
    private static final double BACK_CAMERA_VERTICAL_OFFSET_ANGLE = 30; //degrees

    private static final double FRONT_CAMERA_HORIZONTAL_OFFSET = 11.875; //inches
    private static final double FRONT_CAMERA_HEIGHT = 11.125;
    private static final double FRONT_CAMERA_VERTICAL_OFFSET_ANGLE = 30; //degrees

    private boolean backCamera; // true for back camera, false for front
    private boolean lowTarget; // true for low vision taget, false for high
    private  double cameraHorizontalOffset = BACK_CAMERA_HORIZONTAL_OFFSET; //inches
    private double verticalCameraOffsetAngle = BACK_CAMERA_VERTICAL_OFFSET_ANGLE; //degrees
    private double cameraHeight = BACK_CAMERA_HEIGHT; //degrees

    public MustangCoprocessor() {
        this(NETWORK_TABLE_KEYS);
    }

    private MustangCoprocessor(String[] keys) {
        entries = new HashMap<String, NetworkTableObject>();
        for(String key : keys){
           entries.put(key, new NetworkTableObject(key));
        }
        wallTarget = new VisionValues(NETWORK_TABLE_KEYS[0]);
        backCamera = true;
        lowTarget = true;
        backLedRing = new Solenoid(RobotMap.PCM_MODULE, RobotMap.BACK_LED_RING);
        backLedRing.set(false);
    }

    private class NetworkTableObject {

        private NetworkTableEntry entry;
        private String key;

        /**
         * The key of the NetworkTableEntry that this Object will be attached to.
         */
        public NetworkTableObject(String key) {
            NetworkTableInstance instance = NetworkTableInstance.getDefault();
            NetworkTable table = instance.getTable(TABLE_NAME);
            entry = instance.getEntry(key);
            table.addEntryListener(key, (table2, key2, entry, value, flags) -> {
                this.entry = entry;
                System.out.println("Changed Value");
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
     * Horizontal Angle to the vision target in degrees, as a PIDSource. Provides the VISION_ERROR_CODE if no value found.
     */
    public VisionValue_PIDSource getHAnglePIDSource() {
        return wallTarget.getHAngle_PIDSource();
    }

    /**
     * Vertical Angle to the vision target in degrees, as a PIDSource. Provides the VISION_ERROR_CODE if no value found.
     */
    public VisionValue_PIDSource getVAnglePIDSource() {
        return wallTarget.getVAngle_PIDSource();
    }

    /*
     * Returns distance to the target from the center of the robot
     */
    public double getDistanceToWallTarget() {
        double depth_offset = getOffsetDepth();
        if(MathUtils.doublesEqual(depth_offset, RobotConstants.VISION_ERROR_CODE)){
            return RobotConstants.VISION_ERROR_CODE;
        }
        double real_depth = depth_offset;
        double phi = Robot.sensors.getAngleToTarget();
        real_depth = real_depth + (cameraHorizontalOffset * Math.tan(Math.toRadians(phi)));
        
        return real_depth;
    }

    /*
     * Return the the depth from the horizontally offsetted camera
     */
    private double getOffsetDepth() {
        double vangle = Math.abs(wallTarget.getVAngle()+verticalCameraOffsetAngle);
        double hangle = wallTarget.getHAngle(); //Not need unless diagonal distance below is needed
        double targetHeight = lowTarget ? LOW_TARGET_HEIGHT : HIGH_TARGET_HEIGHT;
        double offset_depth = (targetHeight - cameraHeight) / Math.tan(Math.toRadians(vangle));
        //offset_depth = offset_depth / Math.cos(Math.toRadians(Math.abs(hangle))); - This finds the diagonal distance
        SmartDashboard.putNumber("Depth To Camera", offset_depth);
        return offset_depth;
    }

    public double getTimestamp() {
        return wallTarget.getTimeStamp();
    }

    /*
     * Returns horizontal angle to the target from the center of the robot
     */
    public double getAngleToWallTarget() {
        double hangle_offset = wallTarget.getHAngle();
        double hangle_offset_radians = Math.toRadians(hangle_offset);
        if(MathUtils.doublesEqual(hangle_offset, RobotConstants.VISION_ERROR_CODE)){
            return RobotConstants.VISION_ERROR_CODE;
        }
        double depth_offset = getOffsetDepth();
        double phi = Robot.sensors.getAngleToTarget();
        double alpha = 90 - hangle_offset - phi;
        double y = (depth_offset * Math.sin(Math.toRadians(90-phi)))/ (Math.sin(Math.toRadians(alpha)));
        double temp = cameraHorizontalOffset*cameraHorizontalOffset + y * y - (2 * cameraHorizontalOffset * y * Math.cos(Math.PI/2 - hangle_offset_radians));
        double realDiagonalToTarget = Math.sqrt(temp);
        double beta = Math.toDegrees(Math.asin((y * Math.sin(Math.PI/2 - hangle_offset_radians) / realDiagonalToTarget)));
        double real_angle = beta-90;
    
        // if(hangle_offset <0)
            // real_angle *= -1; //Angle returned is always negative because of arcsins range so multiply by -1 if hangle was positive
        return (real_angle);
    }

    /*
     * Returns an array containing the calculated vision values - [horizontal angle, distance to target]
     */
    public double[] getVisionValues() {
        double[] values = {getAngleToWallTarget(), getDistanceToWallTarget(), getTimestamp()};
        return values;
    }

    /**
     * @return true if using back camera, false if using front
     */
    public boolean isBackCamera() {
        return backCamera;
    }

    /**
     * Sets which camera to use for vision (front/back)
     * @param back true for back camera, false for front
     */
    public void setCamera(boolean back) {
        backCamera = back;
        if(back) {
            cameraHorizontalOffset = BACK_CAMERA_HORIZONTAL_OFFSET; //inches
            verticalCameraOffsetAngle = BACK_CAMERA_VERTICAL_OFFSET_ANGLE; //degrees
            cameraHeight = BACK_CAMERA_HEIGHT; //degrees
            SmartDashboard.putString("vision-camera", "back");
        }
        else {
            cameraHorizontalOffset = FRONT_CAMERA_HORIZONTAL_OFFSET; //inches
            verticalCameraOffsetAngle = FRONT_CAMERA_VERTICAL_OFFSET_ANGLE; //degrees
            cameraHeight = FRONT_CAMERA_HEIGHT; //degrees
            SmartDashboard.putString("vision-camera", "front");
        }
    }

    /**
     * Sets whether to use vision
     * @param enabled true for vision, false for no vision
     */
    public void useVision(boolean enabled) {
        if(enabled) {
            SmartDashboard.putString("vision-enabled", "enabled");
        }
        else {
            SmartDashboard.putString("vision-enabled", "disabled");
        }
    }

    /**
     * Sets the target for vision (must be set properly to get correct distance)
     * @param lowTarget true for low (everything but rocket ball), false for high (rocket ball)
     */
    public void setTargetHeight(boolean lowTarget) {
        this.lowTarget = lowTarget;
    }
    /**
     * Represents a set of vision data received from the coprocessor containing an array of doubles in the form [hangle, vangle, timestamp]
     */
    public class VisionValues {
        private static final int HANGLE_INDEX = 0, VANGLE_INDEX = 1, TIMESTAMP_INDEX = 2;
        private VisionValue_PIDSource hangle, vangle;
        
        private VisionValues(String keyName) {
            hangle = new VisionValue_PIDSource(keyName, HANGLE_INDEX);
            vangle = new VisionValue_PIDSource(keyName, VANGLE_INDEX);
        }

        public VisionValue_PIDSource getHAngle_PIDSource() {
            return hangle;
        }

        public VisionValue_PIDSource getVAngle_PIDSource() {
            return vangle;
        }

        /**
         * Horizontal Angle to the located target in degrees.
         */
        public double getHAngle() {
            return hangle.pidGet();
        }

        /**
         * Vertical Angle to the located target in degrees.
         */
        public double getVAngle() {
            return vangle.pidGet();
        }

        /**
         * Gets the time stamp of the last vision calculation off the pi.
         */
        public double getTimeStamp() {
            return hangle.getEntry(TIMESTAMP_INDEX);
        }

        /**
         * Returns true if a vision target is able to be located through camera
         */
        public boolean canSeeVisionTarget() {
            return !MathUtils.doublesEqual(hangle.pidGet(), RobotConstants.VISION_ERROR_CODE);
        }
    }

    /**
     * Implements a VisionValue (vertical/horizontal angle) as a PIDSource
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

    public void turnOnBackLedRing() {
        backLedRing.set(true);
    }

    public void turnOffBackLedRing() {
        backLedRing.set(false);
    }

}
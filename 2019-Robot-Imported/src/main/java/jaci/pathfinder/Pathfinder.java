package jaci.pathfinder;


/**
 * The main class of the Pathfinder Library. The Pathfinder Library is used for Motion Profile and Trajectory Generation.
 *
 * This class contains some of the most common methods you will use when doing Motion Profiling
 *
 * @author Jaci
 */
public class Pathfinder {

    /**
     * Convert degrees to radians. This is included here for static imports. In this library, all angle values are
     * given in radians
     * @param degrees the degrees input
     * @return the input in radians
     */
    public static double d2r(double degrees) {
        return Math.toRadians(degrees);
    }

    /**
     * Convert radians to degrees. This is included here for static imports.
     * @param radians the radians input
     * @return the input in degrees
     */
    public static double r2d(double radians) {
        return Math.toDegrees(radians);
    }

    /**
     * Bound an angle (in degrees) to -180 to 180 degrees.
	 * @param angle_degrees an input angle in degrees
	 * @return the bounded angle
     */
    public static double boundHalfDegrees(double angle_degrees) {
        while (angle_degrees >= 180.0) angle_degrees -= 360.0;
        while (angle_degrees < -180.0) angle_degrees += 360.0;
        return angle_degrees;
    }

    

}

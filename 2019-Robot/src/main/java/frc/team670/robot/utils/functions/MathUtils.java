/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

/**
 * Provides useful math-related functions.
 * 
 * @author shaylandias
 */
public class MathUtils {

    // This private constructor makes it so no version of this Object can be instantiated, as we only want it for static functions
    private MathUtils() {}

    /**
     * Compares two doubles for equality, returning true if they are equal. Use this since in Java using '==' does not work to compare doubles.
     */
    public static boolean doublesEqual(double d1, double d2) {
        return doublesEqual(d1, d2, 0.0000001);
    }

    /**
     * Compares two doubles for equality, returning true if they are equal. Use this since in Java using '==' does not work to compare doubles.
     * @param d1
     * @param d2
     * @param comparator The value used to compare the doubles after subtracting (ex. 0.0000001). If working with especially small numbers, you would want
     *  to set this very low since the difference between them may just be small and not an issue with double comparation.
     */
    public static boolean doublesEqual(double d1, double d2, double comparator) {
        return (Math.abs(d1-d2) < comparator);
    }

    /*
     * Continue this file with any other math utilities that may be needed throughout the robot.
     */

}

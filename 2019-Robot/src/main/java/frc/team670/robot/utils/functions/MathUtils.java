/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import frc.team670.robot.constants.RobotConstants;
import jaci.pathfinder.Pathfinder;

/**
 * Provides useful math-related functions.
 * 
 * @author shaylandias, meganchoy, varunjoshi
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

    /**
     * Returns the average value of the input doubles.
     */
    public static double average(double[] nums) {
        if(nums.length == 0) {
            return 0;
        }
        double sum = 0;
        for(int i = 0; i < nums.length; i++) {
            sum += nums[i];
        }
        return sum/nums.length;
    }


    /**
     * Returns the average value of the input doubles.
     */
    public static double average(Double ... nums) {
        if(nums.length == 0) {
            return 0;
        }
        double sum = 0;
        for(int i = 0; i < nums.length; i++) {
            sum += nums[i];
        }
        return sum/nums.length;
    }

    /**
     * Returns the distance (always positive) between two points in a coordinate system.
     * @param x1 The x-coord of point 1
     * @param y1 The y-coord of point 1
     * @param x2 The x-coord of point 2
     * @param y2 The y-coord of point 2
     */
    public static double findDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x1-x2), 2) + Math.pow(y2-y1, 2));
    }

    /**
     * Returns true if the value is within +/- tolerance of target
     * @param tolerance The tolerance (must be positive)
     */
    public static boolean isWithinTolerance(double value, double target, double tolerance) {
        return (value > target - tolerance && value < target + tolerance);
    }

    /**
     * Returns the hypotenuse of a triangle given two legs according to the Pythagorean Theorem
     * 
     * @param leg1 The length of the first leg
     * @param leg2 The length of the second leg
     */
    public static double findHypotenuse(double leg1, double leg2) {
        return Math.sqrt(leg1 * leg1 + leg2 * leg2);
    }

    /*
     * Continue this file with any other math utilities that may be needed throughout the robot.
     */

}

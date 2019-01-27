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
     * Converts a tick value taken from a drive base DIO encoder to inches.
     */
    public static double convertDriveBaseTicksToInches(double ticks) {
       double rotations = ticks / RobotConstants.DIO_TICKS_PER_ROTATION;
       return rotations * Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER;
    }

    /**
     * Converts an inch value into drive base DIO Encoder ticks.
     */
    public static int convertInchesToDriveBaseTicks(double inches) {
        double rotations = inches / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
        return (int)(rotations * RobotConstants.DIO_TICKS_PER_ROTATION);
    }

    /**
     * Gets inches per rotations of a NEO motor on the drive base since SparkMAX encoders work in rotations.
     */
    public static double convertDriveBaseRotationsToInches(double rotations) {
        return RobotConstants.DRIVEBASE_INCHES_PER_ROTATION * rotations;
    }

    /**
     * Gets rotations of a NEO motor on the drive base per a value in inches ince SparkMAX encoders work in rotations.
     */
    public static double convertInchesToDriveBaseRotations(double inches) {
        return inches / RobotConstants.DRIVEBASE_INCHES_PER_ROTATION;
    }

    /**
     * Converts a value of per second of the DriveBase Rounds Per Minute
     */
    public static double convertInchesPerSecondToDriveBaseRoundsPerMinute(double inchesPerSecond) {
        // (Inches/seconds) * (60 seconds/1 minute) * ((2 * Diameter inches)/Rotation)
        return inchesPerSecond * 60 / (Math.PI * RobotConstants.DRIVE_BASE_WHEEL_DIAMETER);
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

    public static double convertElbowDegreesToTicks(double degrees) {
        return (degrees / 360) * RobotConstants.ELBOW_TICKS_PER_ROTATION;
    }

    public static double convertElbowTicksToDegrees(double ticks) {
        return (ticks / RobotConstants.ELBOW_TICKS_PER_ROTATION) * 360;
    }

    public static double convertWristDegreesToTicks(double degrees) {
        return (degrees / 360) * RobotConstants.WRIST_TICKS_PER_ROTATION;
    }

    public static double convertWristTicksToDegrees(double ticks) {
        return -1 * Pathfinder.boundHalfDegrees((ticks / RobotConstants.WRIST_TICKS_PER_ROTATION) * 360); // Multiplied by -1 to flip from (-180,180) to (180,-180)
    }

    public static int convertExtensionInchesToTicks(double inches) { // TODO Dimensional Analysis on this cuz this is wrong!
        return (int)(RobotConstants.EXTENSION_TICKS_PER_MOTOR_ROTATION / inches);
    }

    public static double convertExtensionTicksToInches(double ticks) {
        return ticks / RobotConstants.EXTENSION_TICKS_PER_MOTOR_ROTATION / RobotConstants.EXTENSION_MOTOR_ROTATIONS_PER_INCH;
    }

    public static double convertIntakeDegreesToTicks(double degrees) {
        return 0.0; //TODO set this
    }

    public static double convertIntakeTicksToDegrees(double ticks) {
        return 0.0; //TODO set this
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

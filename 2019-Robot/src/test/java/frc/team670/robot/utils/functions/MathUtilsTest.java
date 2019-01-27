/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import static org.junit.Assert.assertEquals;

import org.junit.Assert;
import org.junit.Test;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * A Test for the MathUtils class. Serves as a test example for functions. Add to this for all other functions added.
 * @author shaylandias
 */
public class MathUtilsTest {

    // This test declares many possible calls of the method MathUtils.doublesEqual and then checks them against what the result should be.
    // If one of the results is not as expected, the test fails, letting us know we need to fix the method.
    @Test
    public void doublesEqualTest() {
        assertEquals(true, MathUtils.doublesEqual(0.0, 0.0));
        assertEquals(false, MathUtils.doublesEqual(0.0, 1.0));
        assertEquals(false, MathUtils.doublesEqual(0.0, -1.0));
        assertEquals(true, MathUtils.doublesEqual(-1.0, -1.0));
        assertEquals(false, MathUtils.doublesEqual(-90124124.0, 90214214.0));
        assertEquals(true, MathUtils.doublesEqual(351.0, 351.0));
        assertEquals(false, MathUtils.doublesEqual(351.0, 361.0));
        assertEquals(false, MathUtils.doublesEqual(-351.0, -371.0));
        assertEquals(true, MathUtils.doublesEqual(-351.0, -351.0));
    }

    @Test
    public void averageTest() {
        assertEquals(6, MathUtils.average(6.0), 0.0001);
        assertEquals(6, MathUtils.average(5.0, 7.0), 0.0001);
        assertEquals(-6.0, MathUtils.average(-6.0), 0.0001);
        assertEquals(-6, MathUtils.average(-5.0, -7.0), 0.0001);
        assertEquals(0, MathUtils.average(0.0), 0.0001);
        assertEquals(0, MathUtils.average(), 0.0001);
    }

    @Test
    public void findDistanceTest() {
        assertEquals(1, MathUtils.findDistance(0, 0, 0, 1), 0.000001);
        assertEquals(1, MathUtils.findDistance(0, 0, 0, -1), 0.000001);
        assertEquals(5, MathUtils.findDistance(0, 0, 3, 4), 0.000001);
        assertEquals(1, MathUtils.findDistance(1, 0, 0, 0), 0.000001);
        assertEquals(1, MathUtils.findDistance(0, 1, 0, 0), 0.000001);
        assertEquals(1, MathUtils.findDistance(0, 0, 1, 0), 0.000001);
    }

    @Test
    public void convertDriveBaseTicksToInchesTest(){
        assertEquals(9.42, MathUtils.convertDriveBaseTicksToInches(2048), 0.1);
        assertEquals(13.805, MathUtils.convertDriveBaseTicksToInches(3000), 0.1);
        assertEquals(8.605, MathUtils.convertDriveBaseTicksToInches(1870), 0.1);
    }

    @Test
    public void convertInchesToDriveBaseTicksTest() {
        assertEquals(10864, MathUtils.convertInchesToDriveBaseTicks(50), 1);
        assertEquals(17383, MathUtils.convertInchesToDriveBaseTicks(80), 1);
        assertEquals(21728, MathUtils.convertInchesToDriveBaseTicks(100), 1);

    }

    @Test
    public void convertDriveBaseRotationsToInchesTest() {
        assertEquals(11.14, MathUtils.convertDriveBaseRotationsToInches(5), 0.1);
        assertEquals(22.29, MathUtils.convertDriveBaseRotationsToInches(10), 0.1);
        assertEquals(44.59, MathUtils.convertDriveBaseRotationsToInches(20), 0.1);
    }

    @Test
    public void convertInchesToDriveBaseRotationsTest() {
        assertEquals(22.425, MathUtils.convertInchesToDriveBaseRotations(50), 0.1);
        assertEquals(18.83, MathUtils.convertInchesToDriveBaseRotations(42), 0.1);
        assertEquals(5.37, MathUtils.convertInchesToDriveBaseRotations(12), 0.1);
    }

    @Test
    public void convertElbowDegreesToTicksTest() {
        assertEquals(0, MathUtils.convertElbowDegreesToTicks(0), 0.1);    
        assertEquals(2048, MathUtils.convertElbowDegreesToTicks(180), 0.1);   
        assertEquals(-1024, MathUtils.convertElbowDegreesToTicks(-90), 0.1);
    }
    @Test
    public void convertElbowTicksToDegreesTest() {
        assertEquals(0, MathUtils.convertElbowTicksToDegrees(0), 0.1);    
        assertEquals(-90, MathUtils.convertElbowTicksToDegrees(-1024), 0.1);   
        assertEquals(90, MathUtils.convertElbowTicksToDegrees(1024), 0.1);
    }
    @Test
    public void convertWristDegreesToTicksTest() {
        assertEquals(1024, MathUtils.convertWristDegreesToTicks(90), 0.1);    
        assertEquals(-1024, MathUtils.convertWristDegreesToTicks(-90), 0.1);   
        assertEquals(512, MathUtils.convertWristDegreesToTicks(45), 0.1);
    }

    @Test
    public void convertWristTicksToDegreesTest() {
        assertEquals(90, MathUtils.convertWristTicksToDegrees(1024), 0.1);    
        assertEquals(0, MathUtils.convertWristTicksToDegrees(0), 0.1);   
        assertEquals(180, MathUtils.convertWristTicksToDegrees(2048), 0.1);
    }

    @Test
    public void convertExtensionInchesToTicks() {
        assertEquals(440, MathUtils.convertExtensionInchesToTicks(2), 0.1);    
        assertEquals(1100, MathUtils.convertExtensionInchesToTicks(5), 0.1);   
        assertEquals(713, MathUtils.convertExtensionInchesToTicks(3.243), 0.1);
    }

    @Test
    public void convertExtensionTicksToInches() {
        assertEquals(11.067491897492088, MathUtils.convertExtensionTicksToInches(2435), 0.1);    
        assertEquals(-1.0499345496183459, MathUtils.convertExtensionTicksToInches(-231), 0.1);   
        assertEquals(9.308510639040573, MathUtils.convertExtensionTicksToInches(2048), 0.1);
    }

    /*
     * Add in more tests for all Util methods here.
     */ 

}

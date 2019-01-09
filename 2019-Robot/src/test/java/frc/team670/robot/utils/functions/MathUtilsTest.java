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

    /*
     * Add in more tests for all Util methods here.
     */ 

}

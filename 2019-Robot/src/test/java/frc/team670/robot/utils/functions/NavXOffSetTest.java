/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import static org.junit.Assert.assertEquals;

import org.junit.Test;


/**
 * Add your docs here.
 */
public class NavXOffSetTest {


    // This test declares many possible calls of the method MathUtils.doublesEqual and then checks them against what the result should be.
    // If one of the results is not as expected, the test fails, letting us know we need to fix the method.
    @Test
    public void getYawOffSetTest() {
        assertEquals(-125, getYawOffSet(-170, -45), 0);
        assertEquals(90, getYawOffSet(90.0, 0), 0);
        assertEquals(-135, getYawOffSet(-45, 90), 0);
        assertEquals(135, getYawOffSet(-135, 90), 0);
        assertEquals(178, getYawOffSet(-5, 177), 0);


    }

    /**
     * @return The angle after offSet
     */
    public double getYawOffSet(double rawAngle, double offSet) {
        double rtrnAngle = rawAngle - offSet;
        if (rtrnAngle > 180) { 
            rtrnAngle = rtrnAngle - 360; // returns the same angle but in range [-180, 180]
        }
        else if (rtrnAngle < -180) {
            rtrnAngle = rtrnAngle + 360; 
        }
        return rtrnAngle;
    }
}

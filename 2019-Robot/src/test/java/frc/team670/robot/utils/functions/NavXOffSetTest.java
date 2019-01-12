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

    private double offSet;
    @Test
    private void getYawOffSetTest() {

        setOffSetAngle(45.0);
        assertEquals(45.0, getYawOffSet(90.0), 1);
    }

    public void setOffSetAngle(double x) {
        offSet = x;
    }

    /**
     * @return The angle after offSet
     */
    public double getYawOffSet(double angle) {
        double rtrnAngle = angle - offSet;
        if (rtrnAngle > 180) { 
            rtrnAngle = rtrnAngle - 360; // returns the same angle but in range [-180, 180]
        }
        else if (rtrnAngle < -180) {
            rtrnAngle = rtrnAngle + 360; 
        }
        return rtrnAngle;
    }
}

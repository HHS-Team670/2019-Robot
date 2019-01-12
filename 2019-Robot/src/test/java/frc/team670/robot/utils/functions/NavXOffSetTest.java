/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.utils.functions;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import frc.team254.lib.util.drivers.NavX;

/**
 * Add your docs here.
 */
public class NavXOffSetTest {

    @Test

    private void getYawOffSetTest() {

        NavX.setOffSetAngle(45.0);
        assertEquals(45.0, NavX.getYawOffSet(90.0), 1);
    }
}

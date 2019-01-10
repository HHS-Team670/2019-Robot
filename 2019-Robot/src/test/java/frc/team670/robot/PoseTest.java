/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import frc.team670.robot.Pose;

/**
 * A Test for the MathUtils class. Serves as a test example for functions. Add to this for all other functions added.
 * @author shaylandias
 */
public class PoseTest {

    // This test declares many possible calls of the method getPose and then checks them against what the result should be.
    // If one of the results is not as expected, the test fails, letting us know we need to fix the method.
    @Test
    public void getPoseTest() {

        Pose poseTest = new Pose(0, 0, 0);
        poseTest.update(100, 100, 60);
        assertEquals(86,  poseTest.getPosX(), 1);
        assertEquals(50,  poseTest.getPosY(), 1);
    }


    /*
     * Add in more tests for all Util methods here.
     */ 

}

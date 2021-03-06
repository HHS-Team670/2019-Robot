/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.dataCollection;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.team670.robot.dataCollection.Pose;

/**
 * A Test for the MathUtils class. Serves as a test example for functions. Add to this for all other functions added.
 * @author shaylandias
 */
public class PoseTest {

    // This test declares many possible calls of the method getPose and then checks them against what the result should be.
    // If one of the results is not as expected, the test fails, letting us know we need to fix the method.
    @Test
    public void getPoseTest() {

        Pose poseTest = new Pose(0d, 0d, 0d, 0, 0, 0, 0);
        poseTest.update(100, 100, 60, 0, 0);
        assertEquals(86.0,  poseTest.getPosX(), 0.0001);
        assertEquals(49.0,  poseTest.getPosY(), 0.0001);
        assertEquals(60.0,  poseTest.getRobotAngle(), 0.0001);
        //poseTest = new Pose(0d, 0d, 0d, 0, 0, 0, 0);
        poseTest.update(-100, -100, 0, 0, 0);
        assertEquals(-87.0,  poseTest.getPosX(), 0.0001);
        assertEquals(-50,  poseTest.getPosY(), 0.0001);
        poseTest.update(0, 0, -40, 0, 0);
        assertEquals(-40.0,  poseTest.getRobotAngle(), 0.0001);

        // Angle tests
        // poseTest = new Pose(0d, 0d, 90d, 0, 0, 0, 0); // Never going to happen b/c update is called continuously
        // poseTest.update(0, 0, -90, 0, 0);
        Pose poseTest2 = new Pose(0d, 0d, 160d, 0, 0, 0, 0);
        poseTest2.update(0, 0, -170d, 0, 0);
        assertEquals(175, poseTest2.averagedAngle(), 0.0001);

        //starts at -170
        poseTest.update(0, 0, 170, 0, 0);
        poseTest.update(0, 0, -160, 0, 0);
        assertEquals(-175, poseTest.averagedAngle(), 0.0001);

        poseTest.update(0, 0, 40, 0, 0);
        poseTest.update(0, 0, -50, 0, 0);
        assertEquals(-5, poseTest.averagedAngle(), 0.0001);

        poseTest.update(0, 0, -175, 0, 0);
        poseTest.update(0, 0, 176, 0, 0);
        assertEquals(-179.5, poseTest.averagedAngle(), 0.0001);

        // √µ the square root of the coefficienct of friction of Pokémon
    }


    /*
     * Add in more tests for all Util methods here.
     */ 

}

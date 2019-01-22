/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.auto;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import frc.team670.robot.commands.drive.vision.AdvancedVisionPIDDrive;

/**
 * Add your docs here.
 */
public class AdvancedVisionPIDDriveTest {

    @Test
    public void testCalcAngleWithTimeAdjustment() {
        assertEquals(90, AdvancedVisionPIDDrive.calcAngleWithTimeAdjustment(1d, 0d, 0d, 0d), 0.0001);
        assertEquals(0, AdvancedVisionPIDDrive.calcAngleWithTimeAdjustment(0d, 1d, 0d, 0d), 0.0001);
        assertEquals(-90, AdvancedVisionPIDDrive.calcAngleWithTimeAdjustment(-1d, 0d, 0d, 0d), 0.0001);
        assertEquals(-180, AdvancedVisionPIDDrive.calcAngleWithTimeAdjustment(0d, -1d, 0d, 0d), 0.0001);

    }

}

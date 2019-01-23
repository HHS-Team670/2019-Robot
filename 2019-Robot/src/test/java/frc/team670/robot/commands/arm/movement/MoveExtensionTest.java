/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

/**
 * Add your docs here.
 */
public class MoveExtensionTest {

    private TestExtension extension;
    private double targetDistance;

    public MoveExtensionTest(TestExtension extension, double targetDistance) {
        this.extension = extension;
        this.targetDistance = targetDistance;
    }

    public void move() {
        extension.setDistance(targetDistance);
    }

}

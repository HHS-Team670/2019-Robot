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
public class MoveElbowTest {

    private TestElbow elbow;
    private double targetAngle;

    public MoveElbowTest(TestElbow elbow, double targetAngle) {
        this.elbow = elbow;
        this.targetAngle = targetAngle;
    }

    public void move() {
        elbow.setAngle(targetAngle);
    }
}

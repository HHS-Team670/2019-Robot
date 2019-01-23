/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.Wrist;

/**
 * Add your docs here.
 */
public class TestWrist extends Wrist{

    private double angle;

    public TestWrist(double angle) {
        this.angle = angle;
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * Doesn't actually set MotionMagicSetpoint, instead moves the angle to that point.
     */
    @Override
    public void setMotionMagicSetpoint(double wristAngle) {
        setAngle(wristAngle);
    }

}

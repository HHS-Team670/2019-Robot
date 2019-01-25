/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.Elbow;

/**
 * Add your docs here.
 */
public class TestElbow extends Elbow {


    private double angle;

    public TestElbow() {

    }

    /**
     * Doesn't actually set MotionMagicSetpoint, instead moves the angle to that point.
     */
    @Override
    public void setMotionMagicSetpoint(double wristAngle) {
        angle = wristAngle;
    }

    @Override
    public void initializeMotionmagic() {
        
    }

    @Override
    public double getAngle() {
        return angle;
    }

}

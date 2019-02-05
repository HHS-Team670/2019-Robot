/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.elbow.Elbow;

/**
 * Add your docs here.
 */
public class TestElbow extends BaseElbow {


    private double elbowTicks;

    public TestElbow() {

    }

    /**
     * Doesn't actually set MotionMagicSetpoint, instead moves the angle to that point.
     */
    @Override
    public void setMotionMagicSetpoint(double elbowTicks) {
        this.elbowTicks = elbowTicks;
    }

    @Override
    public double getAngle() {
        return Elbow.convertElbowTicksToDegrees(elbowTicks);
    }

    @Override
    public void setOutput(double output) {
        
    }

    @Override
    public double getOutputCurrent() {
        return 0;
    }

    @Override
    public void setClimbingCurrentLimit() {

    }

    @Override
    public void setNormalCurrentLimit() {

    }

    @Override
    public int getPositionTicks() {
        return (int)elbowTicks;
    }

    @Override
    public boolean isForwardLimitPressed() {
        return false;
    }

    @Override
    public boolean isReverseLmitPressed() {
        return false;
    }
    
    @Override
    public void zero(double encoderValue) {

    }

    @Override
    public double getEncoderValue() {
        return 0;
    }

    @Override
    public void setCurrentControl(int current) {
    }

    @Override
    public void updateArbitraryFeedForward() {

    }

    @Override
    public void setMotionMagicSetpointTicks(int ticks) {

    }


}

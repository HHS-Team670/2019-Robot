/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.elbow.BaseElbow;

/**
 * Add your docs here.
 */
public class TestElbow extends BaseElbow {

    private static final int OFFSET_FROM_ENCODER_ZERO = 0;

    private double elbowAngle;

    public TestElbow(){
        super(null, 0.0, 0, 0, false, 0, 0, 0, 0, 0);
    }

    @Override
    public double getAngleInDegrees() {
        return elbowAngle;
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
    public boolean isForwardLimitPressed() {
        return false;
    }

    @Override
    public boolean isReverseLmitPressed() {
        return false;
    }

    @Override
    public void setCurrentControl(int current) {
    }

    @Override
    public void updateArbitraryFeedForward() {

    }

    @Override
    public double getArbitraryFeedForwardAngleMultiplier() {
        return 0;
    }

    @Override
    public void setQuadratureEncoder(double encoderValue) {

    }

    @Override
    public void setMotionMagicSetpointAngle(double angle) {
        elbowAngle = angle;
    }

}

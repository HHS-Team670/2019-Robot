/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.wrist.BaseWrist;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.subsystems.wrist.Wrist;

/**
 * Add your docs here.
 */
public class TestWrist extends BaseWrist{

    private double wristAngle;

    public TestWrist() {
        super(null, 0.0, 0, 0, false, 0, 0, 0, 0);
    }

    /**
     * Doesn't actually set MotionMagicSetpoint, instead moves the angle to that point.
     */
    @Override
    public void setMotionMagicSetpointAngle(double wristAngle) {
        this.wristAngle = wristAngle;
    }

    @Override
    public double getAngleInDegrees() {
        return wristAngle;
    }

    @Override
    public void enableCurrentLimit() {
    }

    @Override
    public void disableCurrentLimit() {
    }

    @Override
    public void setOutput(double output){
    }

    @Override
    public boolean isForwardLimitPressed() {
        //drive until switch is closed
        return false;
    }
    
    @Override
    public boolean isReverseLimitPressed() {
        //drive until switch is closed
        return false;
    }
    
    @Override
    public void setQuadratureEncoder(double encoderValue) {
    }

    @Override
    public void updateArbitraryFeedForward() {

    }

    @Override
    public double getArbitraryFeedForwardAngleMultiplier() {
        return 0;
    }

}

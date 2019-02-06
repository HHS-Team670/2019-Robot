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

    private double wristTicks;

    public TestWrist() {

    }

    /**
     * Doesn't actually set MotionMagicSetpoint, instead moves the angle to that point.
     */
    @Override
    public void setMotionMagicSetpoint(double wristTicks) {
        this.wristTicks = wristTicks;
    }

    @Override
    public double getAngle() {
        return Wrist.convertWristTicksToDegrees((int)wristTicks);
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
    public int getPositionTicks() {
        return (int)wristTicks;
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
    public void zero(double encoderValue) {
    }

    @Override
    public void updateArbitraryFeedForward() {

    }

    @Override
    public void setMotionMagicSetpointTicks(int ticks) {

    }

    @Override
    public double getAbsoluteAngleMultiplier() {
        return 0;
    }

}

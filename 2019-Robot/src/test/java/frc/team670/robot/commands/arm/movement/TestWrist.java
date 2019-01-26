/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import frc.team670.robot.subsystems.wrist.BaseWrist;

/**
 * Add your docs here.
 */
public class TestWrist extends BaseWrist{

    private double angle;

    public TestWrist() {

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
        return 0;
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

}

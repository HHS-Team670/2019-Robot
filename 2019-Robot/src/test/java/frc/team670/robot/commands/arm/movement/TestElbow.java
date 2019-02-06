/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.subsystems.elbow.BaseElbow;
import frc.team670.robot.subsystems.elbow.Elbow;

/**
 * Add your docs here.
 */
public class TestElbow extends BaseElbow {


    private double elbowAngle;

    public TestElbow(TalonSRX rotatorTalon, double arbitrary_feedforward_constant, int forward_soft_limit, int reverse_soft_limit, boolean timeout, int QUAD_ENCODER_MIN, int QUAD_ENCODER_MAX, int CONTINUOUS_CURRENT_LIMIT, int PEAK_CURRENT_LIMIT){
        super(rotatorTalon, arbitrary_feedforward_constant, forward_soft_limit, reverse_soft_limit, timeout, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT);
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

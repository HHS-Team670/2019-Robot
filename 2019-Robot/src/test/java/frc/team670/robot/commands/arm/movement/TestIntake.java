/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Add your docs here.
 */
public class TestIntake extends BaseIntake {

    private double intakeTicks;

    public TestIntake() {
        super();
    }

    @Override
    public void initDefaultCommand() {

    }

    public void setMotionMagicSetpoint(double intakeTicks) {
        this.intakeTicks = intakeTicks;
    }

    /**
     * Should return the setpoint for the motion magic on the base motor
     */
    public double getMotionMagicSetpoint() {
        return intakeTicks;
    }

    public void setRotatorNeutralMode(NeutralMode mode) {

    }

    /**
     * Returns the tick value of the base motor
     */
    public int getIntakePositionInTicks() {
        return (int) intakeTicks;
    }

    /**
     * Returns the intake angle in degrees
     */
    public double getIntakeAngleInDegrees() {
        return MathUtils.convertIntakeTicksToDegrees(intakeTicks);
    }


    /**
     * Runs the intake at a given percent power
     * 
     * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
     */
    public void runIntake(double power) {

    }

}

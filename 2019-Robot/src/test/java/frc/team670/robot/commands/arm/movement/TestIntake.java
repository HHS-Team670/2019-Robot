/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.arm.movement;

import java.awt.geom.Point2D;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.team670.robot.subsystems.BaseIntake;
import frc.team670.robot.subsystems.Intake;

/**
 * Add your docs here.
 */
public class TestIntake extends BaseIntake {

    private double intakeAngle;

    public TestIntake() {
        super(null, 0.0, 0, 0, true, 0, 0, 0, 0, 0);
    }

    @Override
    public void initDefaultCommand() {

    }

    public void setMotionMagicSetpointAngle(double intakeAngle) {
        this.intakeAngle = intakeAngle;
    }

    /**
     * Should return the setpoint for the motion magic on the base motor
     */
    public double getMotionMagicSetpoint() {
        return intakeAngle;
    }

    public void setRotatorNeutralMode(NeutralMode mode) {

    }

    /**
     * Returns the intake angle in degrees
     */
    public double getAngleInDegrees() {
        return intakeAngle;
    }

    @Override
    /**
     * Runs the intake at a given percent power
     * 
     * @param percentOutput The desired percent power for the rollers to run at [-1, 1]
     */
    public void runIntake(double power, boolean i) {

    }

    /**
     * Returns the x, y coordinates of the top of the intake
     */
    public Point2D.Double getIntakeCoordinates() {
        double x = Intake.INTAKE_ROTATING_LENGTH_IN_INCHES * Math.cos(Math.toRadians(getAngleInDegrees()));
        double y = Intake.INTAKE_FIXED_LENGTH_IN_INCHES + Intake.INTAKE_ROTATING_LENGTH_IN_INCHES * Math.sin(Math.toRadians(getAngleInDegrees()));
        return new Point2D.Double(x, y);
    }

    /**
     * Should return the setpoint coordinates for the motion magic on the base motor
     */
    public Point2D.Double getMotionMagicDestinationCoordinates(){
        return null;
    }

    @Override
    public void updateArbitraryFeedForward() {

    }

    @Override
    public int getPositionTicks() {
        return 0;
    }


    @Override
    public double getArbitraryFeedForwardAngleMultiplier() {
        return 0;
    }

}

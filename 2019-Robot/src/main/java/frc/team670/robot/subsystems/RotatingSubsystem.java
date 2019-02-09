/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Superclass for intake, elbow, wrist, and extension
 */
public abstract class RotatingSubsystem extends Subsystem implements TunableSubsystem {
    protected static final int NO_SETPOINT = 99999;
    protected TalonSRX rotatorTalon;
    protected int setpoint;
    protected boolean timeout;
    protected double arbitraryFeedForwardConstant;

    protected SensorCollection rotatorSensorCollection;
    protected static final int TICKS_PER_ROTATION = 4096;

    public RotatingSubsystem(TalonSRX rotatorTalon, double arbitraryFeedForwardConstant, int forwardSoftLimit, int reverseSoftLimit, boolean timeout, int quadEncoderMin, int quadEncoderMax, int continuousCurrentLimit, int peakCurrentLimit) {
        // For testing purposes
        if (rotatorTalon != null) {
            this.rotatorTalon = rotatorTalon;
            this.rotatorSensorCollection = rotatorTalon.getSensorCollection();
            this.arbitraryFeedForwardConstant = arbitraryFeedForwardConstant;
            this.timeout = timeout;

            setpoint = RotatingSubsystem.NO_SETPOINT;

            int pulseWidthPos = getRotatorPulseWidth() & 4095;

            if (pulseWidthPos < quadEncoderMin) {
                pulseWidthPos += 4096;
            }
            if (pulseWidthPos > quadEncoderMax) {
                pulseWidthPos -= 4096;
            }

            rotatorSensorCollection.setQuadraturePosition(pulseWidthPos, 0);

            rotatorTalon.configContinuousCurrentLimit(continuousCurrentLimit);
            rotatorTalon.configPeakCurrentLimit(peakCurrentLimit);
            rotatorTalon.enableCurrentLimit(true);

            // These thresholds stop the motor when limit is reached
            rotatorTalon.configForwardSoftLimitThreshold(forwardSoftLimit);
            rotatorTalon.configReverseSoftLimitThreshold(reverseSoftLimit);

            // Enable Safety Measures
            rotatorTalon.configForwardSoftLimitEnable(true);
            rotatorTalon.configReverseSoftLimitEnable(true);
        }
    }

    public void zeroPulseWidthEncoder() {
        rotatorSensorCollection.setPulseWidthPosition(0, 0);
    }

    /**
     * Gets the boolean to decide whether or not to pulse or stall the motor
     */
    public boolean getTimeout(){
        return timeout;
    }

    /**
     * Enbales the main talon to percent output mode
     */
    public void enablePercentOutput() {
        rotatorTalon.set(ControlMode.PercentOutput, 0);
    }

    /**
     * Removes the setpoint for the talon on this subsystem
     */
    public void clearSetpoint() {
        setpoint = NO_SETPOINT;
    }


    /**
     * Rotates the talon at a certain percent output
     */
    public void rotatePercentOutput(double output) {
        rotatorTalon.set(ControlMode.PercentOutput, output);
    }

    /**
     * Updates the arbitrary feed forward on this subsystem
     */
    public void updateArbitraryFeedForward(){
        if(setpoint != NO_SETPOINT) {
            double value = getArbitraryFeedForwardAngleMultiplier() * arbitraryFeedForwardConstant;
            rotatorTalon.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
          }
    }

    private int getRotatorPulseWidth(){
        return rotatorSensorCollection.getPulseWidthPosition();
    }

    protected double getMotionMagicSetpoint(){
        return rotatorTalon.getClosedLoopTarget();
    }

    protected int getPositionTicks(){
        return rotatorSensorCollection.getQuadraturePosition();
    }

    /**
     * Gets the multiplier for updating the arbitrary feed forward based on angle and subsystem
     */
    protected abstract double getArbitraryFeedForwardAngleMultiplier();

     /**
     * Sets the setpoint for motion magic (in ticks)
     */
    public abstract void setMotionMagicSetpointAngle(double angle);

    public abstract double getAngleInDegrees();


}

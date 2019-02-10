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
    protected double ARBITRARY_FEEDFORWARD_CONSTANT;
    protected int offsetFromEncoderZero;

    protected SensorCollection rotatorSensorCollection;
    protected static final int TICKS_PER_ROTATION = 4096;

    public RotatingSubsystem(TalonSRX rotatorTalon, double arbitrary_feedforward_constant, int forward_soft_limit, int reverse_soft_limit, boolean timeout, int QUAD_ENCODER_MIN, int QUAD_ENCODER_MAX, int continuous_current_limit, int peak_current_limit, int offsetFromEncoderZero) {
        // For testing purposes

        this.offsetFromEncoderZero = offsetFromEncoderZero;

        rotatorTalon.configFactoryDefault();

        if (rotatorTalon != null) {
            this.rotatorTalon = rotatorTalon;
            this.rotatorSensorCollection = rotatorTalon.getSensorCollection();
            ARBITRARY_FEEDFORWARD_CONSTANT = arbitrary_feedforward_constant;
            this.timeout = timeout;

            setpoint = RotatingSubsystem.NO_SETPOINT;

            int pulseWidthPos = getRotatorPulseWidth() & 4095;

            if (pulseWidthPos < QUAD_ENCODER_MIN) {
                pulseWidthPos += 4096;
            }
            if (pulseWidthPos > QUAD_ENCODER_MAX) {
                pulseWidthPos -= 4096;
            }

            rotatorSensorCollection.setQuadraturePosition(pulseWidthPos, 0);

            rotatorTalon.configContinuousCurrentLimit(continuous_current_limit);
            rotatorTalon.configPeakCurrentLimit(peak_current_limit);
            rotatorTalon.enableCurrentLimit(true);

            // These thresholds stop the motor when limit is reached
        //    rotatorTalon.configForwardSoftLimitThreshold(forward_soft_limit);
        //    rotatorTalon.configReverseSoftLimitThreshold(reverse_soft_limit);

            // Enable Safety Measures
           rotatorTalon.configForwardSoftLimitEnable(false);
           rotatorTalon.configReverseSoftLimitEnable(false);
        }
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
        System.out.println("Intake Put in Percent Output");
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
            double value = getArbitraryFeedForwardAngleMultiplier() * ARBITRARY_FEEDFORWARD_CONSTANT;
            rotatorTalon.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
          }
    }

    protected int getRotatorPulseWidth(){
        return getUnadjustedPulseWidth() - offsetFromEncoderZero;
    }

    protected int getUnadjustedPulseWidth() {
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

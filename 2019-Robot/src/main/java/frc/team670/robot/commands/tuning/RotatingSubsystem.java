/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.commands.tuning;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public abstract class RotatingSubsystem extends Subsystem {
    public static final int NO_SETPOINT = 99999;
    protected TalonSRX rotatorTalon;
    protected int setpoint;
    protected boolean timeout;
    public static final int QUAD_ENCODER_MAX = 890, QUAD_ENCODER_MIN = -1158;
    public double ARBITRARY_FEEDFORWARD_CONSTANT = 0.3; // original 0.22 for one clamp
    public final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in rotations

    private static final int TICKS_PER_ROTATION = 4096;

    public void setTalon(TalonSRX rotatorTalon) {
        this.rotatorTalon = rotatorTalon;
    }

    public abstract void updateArbitraryFeedForward();

    public void enablePercentOutput() {
        rotatorTalon.set(ControlMode.PercentOutput, 0);
    }

    public void clearSetpoint() {
        setpoint = NO_SETPOINT;
    }

    public void rotatePercentOutput(double output) {
        rotatorTalon.set(ControlMode.PercentOutput, output);
    }

    public double getAngle() {
        return (rotatorTalon.getSensorCollection().getQuadraturePosition() / 360) * TICKS_PER_ROTATION;
    }


    public abstract int getPositionTicks();

    public abstract void setMotionMagicSetpointTicks(int ticks);

    @Override
    protected void initDefaultCommand() {

    }

}

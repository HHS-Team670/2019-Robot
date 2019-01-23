/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.commands.arm.JoystickExtension;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Controls motors for motion of extension
 */
public class Extension extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX extensionMotor;
  private double extensionLength;
  public static final double MAX_EXTENSION_BACK = 0; //TODO find this
  public static final double MAX_EXTENSION_FORWARD = 0; //TODO find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  private static final int POSITION_SLOT = 0;
  private final double P = 0.1, I = 0.0, D = 0.0, F = 0.0, RAMP_RATE = 0.15;
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in rotations
  private final int CONTINUOUS_CURRENT_LIMIT = 10, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  public Extension() {
    extensionMotor = new TalonSRX(RobotMap.ARM_EXTENSION_MOTOR);   
    extensionMotor.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		extensionMotor.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		extensionMotor.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		extensionMotor.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    extensionMotor.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    extensionMotor.configMotionCruiseVelocity(RobotConstants.MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
		extensionMotor.configMotionAcceleration(RobotConstants.MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
 
    // These thresholds stop the motor when limit is reached
    extensionMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    extensionMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    extensionMotor.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);

    // Enable Safety Measures
    extensionMotor.configForwardSoftLimitEnable(true);
    extensionMotor.configReverseSoftLimitEnable(true);
    extensionMotor.enableCurrentLimit(true);
    extensionMotor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);
  }

  /**
   * Sets the peak current limit for the elbow motor.
   * @param current Current in amps
   */
  public void setCurrentLimit(int current) {
    extensionMotor.configPeakCurrentLimit(RobotConstants.PEAK_AMPS, RobotConstants.TIMEOUT_MS); // Peak Limit at 0
    extensionMotor.configPeakCurrentDuration(RobotConstants.PEAK_TIME_MS, RobotConstants.TIMEOUT_MS); // Duration at over peak set to 0
    extensionMotor.configContinuousCurrentLimit(current, RobotConstants.TIMEOUT_MS);
  }

  public void enableCurrentLimit() {
    extensionMotor.enableCurrentLimit(true);
  }

  public void disableCurrentLimit() {
    extensionMotor.enableCurrentLimit(false);
  }

  public void setOutput(double output){
    extensionMotor.set(ControlMode.PercentOutput, output);
  }

  /**
   * Gets the current Extension length in absolute ticks with 0 at no extension.
   */
  public int getLengthTicks() {
    return extensionMotor.getSensorCollection().getQuadraturePosition();
  }
  
  /**
   * Gets the current Extension length in absolute inches with 0 at no extension.
   */
  public double getLengthInches() {
    return MathUtils.convertExtensionTicksToInches(getLengthTicks());
  }

  /**
   * Enables the PID Controller for extension
   */
  public void enableExtensionPIDController() {
    SettingUtils.initTalonPID(extensionMotor, POSITION_SLOT, P, I, D, F, -RobotConstants.DEFAULT_EXTENSION_POWER,
        RobotConstants.DEFAULT_EXTENSION_POWER, FeedbackDevice.CTRE_MagEncoder_Relative, RAMP_RATE);
        extensionMotor.selectProfileSlot(POSITION_SLOT, 0);
  }

  /**
   * Modifies the setpoint for the PID Controller
   */
  public void setPIDControllerSetpoint(int setpoint) {
    extensionMotor.set(ControlMode.Position, setpoint);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new JoystickExtension());
  }

  /**
   * @return true if forward limit switch closed, false if not
   */
  public boolean getForwardLimitSwitch() {
    //drive until switch is closed
    return extensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  public boolean getReverseLimitSwitch() {
    //drive until switch is closed
    return extensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  /**
   * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
   */
  public void resetExtension(double encoderValue) {
    extensionMotor.getSensorCollection().setQuadraturePosition((int)encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  /**
   * @return the current encoder value of the extension motor
   */
  public double getEncoderValue() {
    return extensionMotor.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Selects the PID Slot dedicated to MotionMagic to give it the correct PID Values
   */
  public void initializeMotionmagic() {
    extensionMotor.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
  }

  /**
   * Setup for movement and Motion Magic
   */
  public void setMotionMagicSetpoint(double extensionLength) {
    extensionMotor.set(ControlMode.MotionMagic, extensionLength);
  }


}

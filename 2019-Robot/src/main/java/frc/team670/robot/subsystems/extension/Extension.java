/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.extension;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.commands.arm.joystick.JoystickExtension;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Arm;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Controls motors for motion of extension
 */
public class Extension extends BaseExtension {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX extensionMotor;

  public static int EXTENSION_IN_POS = 0; // TODO Set These
  public static int EXTENSION_OUT_POS = 12000;

  public static final double MAX_EXTENSION_BACK = 0; //TODO find this
  public static final double MAX_EXTENSION_FORWARD = 0; //TODO find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  private static final int POSITION_SLOT = 0;
  private final double P = 0.1, I = 0.0, D = 0.0, F = 0.0, RAMP_RATE = 0.1;
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private final int FORWARD_SOFT_LIMIT = EXTENSION_IN_POS - 100, REVERSE_SOFT_LIMIT = EXTENSION_OUT_POS + 100; // TODO figure out the values in rotations
  public static final int EXTENSION_ENCODER_OUT = 0;

  private final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  private static final double EXTENSION_POWER = 0.75;

  private static int EXTENSION_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
  private static int EXTENSION_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this


  public Extension() {
    extensionMotor = new TalonSRX(RobotMap.ARM_EXTENSION_MOTOR);   
    super.setTalon(extensionMotor);
    extensionMotor.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		extensionMotor.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		extensionMotor.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		extensionMotor.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    extensionMotor.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    extensionMotor.configMotionCruiseVelocity(EXTENSION_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
		extensionMotor.configMotionAcceleration(EXTENSION_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
 
    // These thresholds stop the motor when limit is reached
    extensionMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    extensionMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    extensionMotor.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    extensionMotor.configClosedloopRamp(0);
    extensionMotor.configOpenloopRamp(RAMP_RATE);

    // Enable Safety Measures
    extensionMotor.configForwardSoftLimitEnable(true);
    extensionMotor.configReverseSoftLimitEnable(true);
    extensionMotor.enableCurrentLimit(true);
    extensionMotor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);

    extensionMotor.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    extensionMotor.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    extensionMotor.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    extensionMotor.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    //Tuning stuff
    setpoint = NO_SETPOINT;

    int pulseWidthPos = getExtensionPulseWidth()&4095;

    if (pulseWidthPos < QUAD_ENCODER_MIN) {
      pulseWidthPos += 4096;
    } 
    if (pulseWidthPos > QUAD_ENCODER_MAX) {
      pulseWidthPos -= 4096;
    }

    extensionMotor.getSensorCollection().setQuadraturePosition(pulseWidthPos, 0);

    extensionMotor.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    extensionMotor.configPeakCurrentLimit(0);
    extensionMotor.enableCurrentLimit(true);

    timeout = false;

    enablePercentOutput();
  }

  @Override
  public void setCurrentLimit(int current) {
    extensionMotor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT); // Peak Limit at 0
    extensionMotor.configPeakCurrentDuration(0); // Duration at over peak set to 0
    extensionMotor.configContinuousCurrentLimit(current);
  }

  @Override
  public void enableCurrentLimit() {
    extensionMotor.enableCurrentLimit(true);
  }

  @Override
  public void disableCurrentLimit() {
    extensionMotor.enableCurrentLimit(false);
  }

  @Override
  public void setOutput(double output){
    extensionMotor.set(ControlMode.PercentOutput, output);
  }

  @Override
  public int getLengthTicks() {
    return extensionMotor.getSensorCollection().getQuadraturePosition();
  }
  
  @Override
  public double getLengthInches() {
    return convertExtensionTicksToInches(getLengthTicks());
  }

  @Override
  public void enableExtensionPIDController() {
    SettingUtils.initTalonPID(extensionMotor, POSITION_SLOT, P, I, D, F, -EXTENSION_POWER,
                              EXTENSION_POWER, FeedbackDevice.CTRE_MagEncoder_Relative, RAMP_RATE);
    extensionMotor.selectProfileSlot(POSITION_SLOT, 0);
  }

  @Override
  public void setPIDControllerSetpoint(int setpoint) {
    extensionMotor.set(ControlMode.Position, setpoint);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new JoystickExtension(this));
  }

  @Override
  public boolean isReverseLimitPressed() {
    //drive until switch is closed
    return extensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public boolean isForwardLimitPressed() {
    //drive until switch is closed
    return extensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  @Override
  public void zero(double encoderValue) {
    extensionMotor.getSensorCollection().setQuadraturePosition((int)encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  @Override
  public void setMotionMagicSetpoint(double extensionTicks) {
    extensionMotor.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
    extensionMotor.set(ControlMode.MotionMagic, extensionTicks);
  }

  /**
   * Converts inches for the intake into ticks
   */
  public static int convertExtensionInchesToTicks(double inches) { 
    //inches * (rotation/inches) * (ticks / rotation)
    return (int)(inches * RobotConstants.EXTENSION_MOTOR_ROTATIONS_PER_INCH * RobotConstants.EXTENSION_TICKS_PER_MOTOR_ROTATION);
  }

  /**
   * Converts ticks for the intake into inches
   */
  public static double convertExtensionTicksToInches(double ticks) {
    //ticks * (rotations/ticks) * (inches / rotations)
    return ticks / RobotConstants.EXTENSION_TICKS_PER_MOTOR_ROTATION / RobotConstants.EXTENSION_MOTOR_ROTATIONS_PER_INCH;
  }

  @Override
  public void setMotionMagicSetpointTicks(int ticks) {
    super.setpoint = ticks;
    extensionMotor.set(ControlMode.MotionMagic, ticks);
  }

  @Override
  public int getPositionTicks(){
    return getLengthTicks();
  }

  @Override
  public void updateArbitraryFeedForward() {
    if(setpoint != NO_SETPOINT) {
      double value = -1.0 * Math.sin(Math.toRadians(Arm.getCurrentState().getElbowAngle())) * ARBITRARY_FEEDFORWARD_CONSTANT;
      extensionMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
    }
  }

  public int getExtensionPulseWidth() {  
    return extensionMotor.getSensorCollection().getPulseWidthPosition();
  }

}

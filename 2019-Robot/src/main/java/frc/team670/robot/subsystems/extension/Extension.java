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

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.SettingUtils;

/**
 * Controls motors for motion of extension
 */
public class Extension extends BaseExtension {

  private TalonSRX extensionMotor;

  public static final double TICKS_PER_MOTOR_ROTATION = 4096; // Should be real value
  public static final double MOTOR_ROTATIONS_PER_INCH = 0.169333; //Should be real value

  // Position PID control constants when doing Arm Climb 
  private static final int POSITION_SLOT = 1;
  private final double P_P = 0.1, P_I = 0.0, P_D = 0.0, P_F = 0.0, RAMP_RATE = 0.1;
  private static final double EXTENSION_POWER = 0.75; // TODO set this for Extension movement when climbing
  private static final int CONTINUOUS_CURRENT_LIMIT = 10, PEAK_CURRENT_LIMIT = 20;

  private static final int START_POSITION_TICKS = 0; // TODO set this. Start position needed since extension has no absolute encoder

  // Motion Magic
  private static final int kPIDLoopIdx = 0, MOTION_MAGIC_SLOT = 0, kTimeoutMs = 0;
  public static final int EXTENSION_IN_POS = 0; // TODO Set These
  public static final int EXTENSION_OUT_POS = 8475; // TODO Set this in ticks
  public static final double EXTENSION_OUT_IN_INCHES = 12.75;
  public static final double FIXED_LENGTH = 19.75;
  public static final int FORWARD_SOFT_LIMIT = EXTENSION_OUT_POS - 100, REVERSE_SOFT_LIMIT = EXTENSION_IN_POS + 100;
 
  private static final double MM_F = 0, MM_P = 0.54, MM_I = 0, MM_D = 0.5; 
  private static final int MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 800; 
  private static final int EXTMOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; 
  public static final int QUAD_ENCODER_MAX = FORWARD_SOFT_LIMIT + 200, QUAD_ENCODER_MIN = REVERSE_SOFT_LIMIT - 200; 

  public static final double ARBITRARY_FEEDFORWARD_CONSTANT = 0.055;
  public static final double MAX_EXTENSION_OUTPUT = 0.4;


  private static final int CLOSED_LOOP_ALLOWABLE_ERROR = 10;
  private double setpoint;
  private static final double NO_SETPOINT = 99999;

  private static final int OFFSET_FROM_ENCODER_ZERO = 0;

  private Notifier warningWhenLimitTripped;


  public Extension() {
    extensionMotor = new TalonSRX(RobotMap.ARM_EXTENSION_MOTOR); 
    
    extensionMotor.configFactoryDefault();
    extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    

    extensionMotor.setInverted(true);
    extensionMotor.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
		extensionMotor.config_kF(MOTION_MAGIC_SLOT, MM_F, kTimeoutMs);
		extensionMotor.config_kP(MOTION_MAGIC_SLOT, MM_P, kTimeoutMs);
		extensionMotor.config_kI(MOTION_MAGIC_SLOT, MM_I, kTimeoutMs);
    extensionMotor.config_kD(MOTION_MAGIC_SLOT, MM_D, kTimeoutMs);
    extensionMotor.configMotionCruiseVelocity(MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
		extensionMotor.configMotionAcceleration(EXTMOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
 

    extensionMotor.configAllowableClosedloopError(MOTION_MAGIC_SLOT, CLOSED_LOOP_ALLOWABLE_ERROR);
    // These thresholds stop the motor when limit is reached
    extensionMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    extensionMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    extensionMotor.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);
    extensionMotor.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);

    extensionMotor.setNeutralMode(NeutralMode.Brake);

    // Enable Safety Measures
    extensionMotor.configForwardSoftLimitEnable(true);
    extensionMotor.configReverseSoftLimitEnable(true);
    extensionMotor.enableCurrentLimit(true);

    extensionMotor.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    extensionMotor.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    extensionMotor.configPeakOutputForward(MAX_EXTENSION_OUTPUT, RobotConstants.kTimeoutMs);
    extensionMotor.configPeakOutputReverse(-MAX_EXTENSION_OUTPUT, RobotConstants.kTimeoutMs);

    extensionMotor.configPeakCurrentDuration(300);

    if(getReverseLimitSwitchTripped()) {
      extensionMotor.setSelectedSensorPosition(EXTENSION_IN_POS);
    }
    else if(getForwardLimitSwitchTripped()) {
      extensionMotor.setSelectedSensorPosition(EXTENSION_OUT_POS);
    }

    extensionMotor.setSelectedSensorPosition(0); // TODO: Remove after limit switches are wired


    //Tuning stuff
    setpoint = NO_SETPOINT;

    // int pulseWidthPos = getExtensionPulseWidth()&4095;

    // if (pulseWidthPos < QUAD_ENCODER_MIN) {
    //   pulseWidthPos += 4096;
    // } 
    // if (pulseWidthPos > QUAD_ENCODER_MAX) {
    //   pulseWidthPos -= 4096;
    // }

    // extensionMotor.getSensorCollection().setQuadraturePosition(pulseWidthPos, 0);
    extensionMotor.getSensorCollection().setQuadraturePosition(START_POSITION_TICKS, 0); // The Extension 
    stop();

    warningWhenLimitTripped = new Notifier(new Runnable() {
      public void run() {
        warnifLimitHit();
      }
    });
    warningWhenLimitTripped.startPeriodic(0.25);

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
  public void enableCoastMode() {
    extensionMotor.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void enableBrakeMode() {
    extensionMotor.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void disableCurrentLimit() {
    extensionMotor.enableCurrentLimit(false);
  }

  @Override
  public void setOutput(double output){
    extensionMotor.set(ControlMode.PercentOutput, output);
  }

  private int getPositionTicks() {
    return extensionMotor.getSelectedSensorPosition(0);
  }

  @Override
  public int getLengthTicks() {
    return getPositionTicks();
  }
  
  @Override
  public double getLengthInches() {
    return convertExtensionTicksToInches(getPositionTicks());
  }

  @Override
  public synchronized void enableExtensionPIDController() {
    SettingUtils.initTalonPID(extensionMotor, POSITION_SLOT, P_P, P_I, P_D, P_F, -EXTENSION_POWER,
                              EXTENSION_POWER, FeedbackDevice.CTRE_MagEncoder_Relative, RAMP_RATE);
    extensionMotor.selectProfileSlot(POSITION_SLOT, 0);
  }

  @Override
  public synchronized void setPIDControllerSetpointInInches(double setpointInInches) {
    setpoint = NO_SETPOINT;
    extensionMotor.set(ControlMode.Position, convertExtensionInchesToTicks(setpointInInches));
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new JoystickExtension(this));
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
  public void setQuadratureEncoder(double encoderValue) {
    extensionMotor.getSensorCollection().setQuadraturePosition((int)encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  @Override
  public synchronized void setMotionMagicSetpointInInches(double extensionSetpointInInches) {
    extensionMotor.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
    setpoint = convertExtensionInchesToTicks(extensionSetpointInInches);
    enableBrakeMode();
    extensionMotor.set(ControlMode.MotionMagic, setpoint);
  }

  public double getMotionMagicSetpoint() {
    return extensionMotor.getClosedLoopTarget();
  }

  /**
   * Converts inches for the intake into ticks
   */
  private static int convertExtensionInchesToTicks(double inches) { 
    //inches * (rotation/inches) * (ticks / rotation)
    return (int)(inches * MOTOR_ROTATIONS_PER_INCH * TICKS_PER_MOTOR_ROTATION);
  }

  /**
   * Converts ticks for the intake into inches
   */
  private static double convertExtensionTicksToInches(double ticks) {
    //ticks * (rotations/ticks) * (inches / rotations)
    return ticks / TICKS_PER_MOTOR_ROTATION / MOTOR_ROTATIONS_PER_INCH;
  }


  /**
   * Updates the arbitrary feed forward on this subsystem
   */
  public synchronized void updateArbitraryFeedForward() {
    if (setpoint != NO_SETPOINT) {
      double value = getArbitraryFeedForwardAngleMultiplier() * ARBITRARY_FEEDFORWARD_CONSTANT;
      extensionMotor.set(ControlMode.MotionMagic, setpoint, DemandType.ArbitraryFeedForward, value);
    }
  }

  public void warnifLimitHit() {
    if (getForwardLimitSwitchTripped())
      SmartDashboard.putString("warning", "EXTENSION FORWARD LIMIT SWITCH TRIPPED");
    if (getReverseLimitSwitchTripped())
      SmartDashboard.putString("warning", "EXTENSION REVERSE LIMIT SWITCH TRIPPED");
  }

  public double getArbitraryFeedForwardAngleMultiplier() {
    double angle = Robot.arm.getElbow().getAngleInDegrees();
    double output = Math.cos(Math.toRadians(angle));
    return output;
  }

  private int getExtensionPulseWidth() {  
    return extensionMotor.getSensorCollection().getPulseWidthPosition();
  }

  public double getForwardSoftLimitInInches(){
    return convertExtensionTicksToInches(FORWARD_SOFT_LIMIT);
  }

  public double getReverseSoftLimitInInches(){
    return convertExtensionTicksToInches(REVERSE_SOFT_LIMIT);
  }

  @Override
  public boolean getTimeout() {
    return false;
  }

  @Override
  public synchronized void stop() {
    setpoint = NO_SETPOINT;
    extensionMotor.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public synchronized void moveByPercentOutput(double output) {
    setpoint = NO_SETPOINT;
    extensionMotor.set(ControlMode.PercentOutput, output);
  }

  public void sendDataToDashboard() {
    SmartDashboard.putNumber("Extension Unadjusted Absolute Ticks", getUnadjustedPulseWidth());
    SmartDashboard.putNumber("Extension Absolute Ticks", getRotatorPulseWidth());
    SmartDashboard.putNumber("Extension Quadrature Ticks", getPositionTicks());
    SmartDashboard.putNumber("Extension Setpoint", setpoint);
  }

  
  private int getRotatorPulseWidth(){
    return getUnadjustedPulseWidth() - OFFSET_FROM_ENCODER_ZERO;
  }

  private int getUnadjustedPulseWidth() {
    return extensionMotor.getSensorCollection().getPulseWidthPosition();
  }

  @Override
  public boolean getReverseLimitSwitchTripped(){
    return extensionMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  @Override
  public boolean getForwardLimitSwitchTripped(){
    return extensionMotor.getSensorCollection().isFwdLimitSwitchClosed();
  }

  private void setSelectedSensorPosition(int sensorPos) {
    extensionMotor.setSelectedSensorPosition(sensorPos);
  }

  public void resetLimitSwitch(){
    if(getForwardLimitSwitchTripped()){
      extensionMotor.setSelectedSensorPosition(Extension.EXTENSION_OUT_POS);
    }

    if(getReverseLimitSwitchTripped()){
      extensionMotor.setSelectedSensorPosition(Extension.EXTENSION_IN_POS);
    }
  }



}

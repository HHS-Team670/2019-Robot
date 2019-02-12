/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems.wrist;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.team670.robot.Robot;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import jaci.pathfinder.Pathfinder;

/**
 * Controls wrist motors
 */
public class Wrist extends BaseWrist {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static double WRIST_START_POS = 6000; //TODO Set this in ticks

  private static final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0;

  // Motion Magic
  private static final int kPIDLoopIdx = 0, MOTION_MAGIC_SLOT = 0, kTimeoutMs = 0;
  private static final double MM_F = 0, MM_P = 0, MM_I = 0, MM_D = 0; //TODO figure out what these are
  private static final double ARBITRARY_FEEDFORWARD = 0; // TODO set this
  private static final int OFFSET_FROM_ENCODER_ZERO = 0; // TODO set this
  private static final int WRIST_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 90; // TODO set this
  private static final int WRIST_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 400; // TODO set this
  public static final int FORWARD_SOFT_LIMIT = 850, REVERSE_SOFT_LIMIT = -940; // TODO set this
  public static final int QUAD_ENCODER_MAX = FORWARD_SOFT_LIMIT + 200, QUAD_ENCODER_MIN = REVERSE_SOFT_LIMIT - 200; //TODO Set these values based on soft limits (especially add/subtract)

  public Wrist() {
    super(new TalonSRX(RobotMap.ARM_WRIST_ROTATION), ARBITRARY_FEEDFORWARD, FORWARD_SOFT_LIMIT, REVERSE_SOFT_LIMIT, false, QUAD_ENCODER_MIN, QUAD_ENCODER_MAX, CONTINUOUS_CURRENT_LIMIT, PEAK_CURRENT_LIMIT, OFFSET_FROM_ENCODER_ZERO);
    rotatorTalon.selectProfileSlot(MOTION_MAGIC_SLOT, kPIDLoopIdx);
		rotatorTalon.config_kF(MOTION_MAGIC_SLOT, MM_F, kTimeoutMs);
		rotatorTalon.config_kP(MOTION_MAGIC_SLOT, MM_P, kTimeoutMs);
		rotatorTalon.config_kI(MOTION_MAGIC_SLOT, MM_I, kTimeoutMs);
    rotatorTalon.config_kD(MOTION_MAGIC_SLOT, MM_D, kTimeoutMs);
    rotatorTalon.configMotionCruiseVelocity(WRIST_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    rotatorTalon.configMotionAcceleration(WRIST_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    
    rotatorTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    rotatorTalon.setNeutralMode(NeutralMode.Brake);

    stop();
  }
  
  @Override
  public void enableCurrentLimit() {
    rotatorTalon.enableCurrentLimit(true);
  }

  @Override
  public void disableCurrentLimit() {
    rotatorTalon.enableCurrentLimit(false);
  }

  @Override
  public void setOutput(double output){
    rotatorTalon.set(ControlMode.PercentOutput, output);
  }
  
  @Override
  public double getAngleInDegrees() {
    return convertWristTicksToDegrees(getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {
    // setDefaultCommand(new JoystickWrist(this));
  }

  @Override
  public boolean isForwardLimitPressed() {
    //drive until switch is closed
    return rotatorTalon.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  @Override
  public boolean isReverseLimitPressed() {
    //drive until switch is closed
    return rotatorTalon.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  @Override
  public void setQuadratureEncoder(double encoderValue) {
    rotatorTalon.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  /**
   * Converts an angle for the wrist into ticks
   */
  private static int convertWristDegreesToTicks(double degrees) {
    //If straight is 0 and going forward is positive
    // percentage * half rotation
    return (int)((degrees / 360) * TICKS_PER_ROTATION);
  }

  /**
   * Converts an angle for the wrist into ticks
   */
  private static double convertWristTicksToDegrees(int ticks) {
    //If straight is 0 and going forward is positive
    // percentage * half degrees rotation
      return ((360 * ticks) / TICKS_PER_ROTATION);
  }

  @Override
  public double getArbitraryFeedForwardAngleMultiplier() {

    double angle = Robot.arm.getElbow().getAngleInDegrees() + getAngleInDegrees();

    Pathfinder.boundHalfDegrees(angle); // In case it wraps around after this addition, binds the angle.

    return -1 * Math.sin(Math.toRadians(angle));
  }

  public int getWristPulseWidth() {
    return rotatorTalon.getSensorCollection().getPulseWidthPosition();
  }

  @Override
  public void setMotionMagicSetpointAngle(double angle) {
    setpoint = convertWristDegreesToTicks(angle);
    rotatorTalon.set(ControlMode.MotionMagic, setpoint);
  }
}
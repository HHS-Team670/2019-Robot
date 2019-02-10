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

import frc.team670.robot.commands.arm.joystick.JoystickWrist;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.subsystems.Arm;

/**
 * Controls wrist motors
 */
public class Wrist extends BaseWrist {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public static double WRIST_START_POS = 6000; //TODO Set this
  public static final double MAX_WRIST_FORWARD = 0; //TODO find this
  public static final double MAX_WRIST_BACK = 0; //TODO find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  private static final int OFFSET_FROM_ENCODER_ZERO = 0;

  public static final int QUAD_ENCODER_MAX = 890, QUAD_ENCODER_MIN = -1158; //TODO Set these values

  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private static int WRIST_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS = 15000; // TODO set this
  private static int WRIST_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS = 6000; // TODO set this

  public Wrist() {
    super(new TalonSRX(RobotMap.ARM_WRIST_ROTATION), 0.0, 0, 0, false, 0, 0, 20, 0, OFFSET_FROM_ENCODER_ZERO);
    rotatorTalon.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		rotatorTalon.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		rotatorTalon.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		rotatorTalon.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    rotatorTalon.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    rotatorTalon.configMotionCruiseVelocity(WRIST_MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    rotatorTalon.configMotionAcceleration(WRIST_MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    
    rotatorTalon.configNominalOutputForward(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configNominalOutputReverse(0, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputForward(1, RobotConstants.kTimeoutMs);
    rotatorTalon.configPeakOutputReverse(-1, RobotConstants.kTimeoutMs);

    rotatorTalon.setNeutralMode(NeutralMode.Brake);

    enablePercentOutput();
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
    return (int)((degrees / 180) * (0.5 * RobotConstants.WRIST_TICKS_PER_ROTATION));
  }

  /**
   * Converts an angle for the wrist into ticks
   */
  private static double convertWristTicksToDegrees(int ticks) {
    //If straight is 0 and going forward is positive
    // percentage * half degrees rotation
    return (ticks / (0.5 * RobotConstants.WRIST_TICKS_PER_ROTATION)) * 180;
  }

  @Override
  public double getArbitraryFeedForwardAngleMultiplier() {
    return (-1.0 * Math.cos(Math.toRadians(getAngleInDegrees() + Arm.getCurrentState().getElbowAngle())));
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
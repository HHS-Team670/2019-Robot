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
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Controls wrist motors
 */
public class Wrist extends BaseWrist {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private TalonSRX wristRotation;
  public static final double MAX_WRIST_FORWARD = 0; //TODO find this
  public static final double MAX_WRIST_BACK = 0; //TODO find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  private static final double RAMP_RATE = 0.1;

  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  private final int FORWARD_SOFT_LIMIT = 0, REVERSE_SOFT_LIMIT = 0; // TODO figure out the values in rotations
  private final int CONTINUOUS_CURRENT_LIMIT = 20, PEAK_CURRENT_LIMIT = 0; // TODO set current limit in Amps

  public Wrist() {
    wristRotation = new TalonSRX(RobotMap.ARM_WRIST_ROTATION); 
    wristRotation.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		wristRotation.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		wristRotation.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		wristRotation.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    wristRotation.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    wristRotation.configMotionCruiseVelocity(RobotConstants.MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    wristRotation.configMotionAcceleration(RobotConstants.MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
    

    // These thresholds stop the motor when limit is reached
    wristRotation.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT);
    wristRotation.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT);
    wristRotation.configContinuousCurrentLimit(CONTINUOUS_CURRENT_LIMIT);

    // Enable Safety Measures
    wristRotation.configForwardSoftLimitEnable(true);
    wristRotation.configReverseSoftLimitEnable(true);
    wristRotation.enableCurrentLimit(true);
    wristRotation.configPeakCurrentLimit(PEAK_CURRENT_LIMIT);

    wristRotation.setNeutralMode(NeutralMode.Brake);

    wristRotation.configClosedloopRamp(RAMP_RATE);
    wristRotation.configOpenloopRamp(RAMP_RATE);

  }
  
  @Override
  public void enableCurrentLimit() {
    wristRotation.enableCurrentLimit(true);
  }

  @Override
  public void disableCurrentLimit() {
    wristRotation.enableCurrentLimit(false);
  }

  @Override
  public void setOutput(double output){
    wristRotation.set(ControlMode.PercentOutput, output);
  }

  @Override
  public int getPositionTicks() {
    return wristRotation.getSensorCollection().getQuadraturePosition();
  }
  
  @Override
  public double getAngle() {
    return MathUtils.convertWristTicksToDegrees(getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {
    setDefaultCommand(new JoystickWrist(this));
  }

  @Override
  public boolean isForwardLimitPressed() {
    //drive until switch is closed
    return wristRotation.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  @Override
  public boolean isReverseLimitPressed() {
    //drive until switch is closed
    return wristRotation.getSensorCollection().isRevLimitSwitchClosed();
  }
  
  @Override
  public void zero(double encoderValue) {
    wristRotation.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  @Override
  public void initializeMotionmagic() {
    wristRotation.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
  }

  @Override
  public void setMotionMagicSetpoint(double wristAngle) {  
    wristRotation.set(ControlMode.MotionMagic, MathUtils.convertWristDegreesToTicks(wristAngle));
  }
}

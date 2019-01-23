/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.team670.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.team670.robot.constants.RobotConstants;
import frc.team670.robot.constants.RobotMap;
import frc.team670.robot.utils.functions.MathUtils;

/**
 * Controls wrist motors
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private TalonSRX wristRotation;
  public static final double MAX_WRIST_FORWARD = 0; //TODO find this
  public static final double MAX_WRIST_BACK = 0; //TODO find this
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotMotionMagic = 0, kTimeoutMs = 0;

  public Wrist() {
    wristRotation = new TalonSRX(RobotMap.ARM_WRIST_ROTATION); 
    wristRotation.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
		wristRotation.config_kF(kSlotMotionMagic, kF, kTimeoutMs);
		wristRotation.config_kP(kSlotMotionMagic, kP, kTimeoutMs);
		wristRotation.config_kI(kSlotMotionMagic, kI, kTimeoutMs);
    wristRotation.config_kD(kSlotMotionMagic, kD, kTimeoutMs);
    wristRotation.configMotionCruiseVelocity(RobotConstants.MOTIONMAGIC_VELOCITY_SENSOR_UNITS_PER_100MS, kTimeoutMs);
		wristRotation.configMotionAcceleration(RobotConstants.MOTIONMAGIC_ACCELERATION_SENSOR_UNITS_PER_100MS, kTimeoutMs);
  }
  
  /**
   * Sets the peak current limit for wrist rotation motor.
   * @param current Current in amps
   */
  public void setCurrentLimit(int current) {
    wristRotation.configPeakCurrentLimit(RobotConstants.PEAK_AMPS, RobotConstants.TIMEOUT_MS); // Peak Limit at 0
    wristRotation.configPeakCurrentDuration(RobotConstants.PEAK_TIME_MS, RobotConstants.TIMEOUT_MS); // Duration at over peak set to 0
    wristRotation.configContinuousCurrentLimit(current, RobotConstants.TIMEOUT_MS);
  }

  public void enableCurrentLimit() {
    wristRotation.enableCurrentLimit(true);
  }

  public void disableCurrentLimit() {
    wristRotation.enableCurrentLimit(false);
  }

  public void setOutput(double output){
    wristRotation.set(ControlMode.PercentOutput, output);
  }

  public int getPositionTicks() {
    return wristRotation.getSensorCollection().getQuadraturePosition();
  }
  
  public double getAngle() {
    return MathUtils.convertWristTicksToDegrees(getPositionTicks());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  /**
  * @return true if the forward limit switch is closed, false if open
  */
  public boolean getForwardLimitSwitch() {
    //drive until switch is closed
    return wristRotation.getSensorCollection().isFwdLimitSwitchClosed();
  }
  
  /**
   * @return true if the forward limit switch is closed, false if open
   */
  public boolean getReverseLimitSwitch() {
    //drive until switch is closed
    return wristRotation.getSensorCollection().isRevLimitSwitchClosed();
  }
  


  /**
  * Sets the SensorCollection encoder value to encoderValue (use this to reset the encoder when at a known position)
  */
  public void resetWrist(double encoderValue) {
    wristRotation.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  /**
   * @return the current encoder value of the wrist motor
   */
  public double getEncoderValue() {
    return wristRotation.getSensorCollection().getQuadraturePosition();
  }

  /**
   * Selects the PID Slot dedicated to MotionMagic to give it the correct PID Values
   */
  public void initializeMotionmagic() {
    wristRotation.selectProfileSlot(kSlotMotionMagic, kPIDLoopIdx);
  }

  /**
   * Setup for movement and Motion Magic
   */
  public void setMotionMagicSetpoint(double wristAngle) {  
    wristRotation.set(ControlMode.MotionMagic, MathUtils.convertWristDegreesToTicks(wristAngle));
  }
}

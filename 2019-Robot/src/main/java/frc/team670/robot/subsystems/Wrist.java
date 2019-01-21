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

/**
 * Controls wrist motors
 */
public class Wrist extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  private TalonSRX wristRotation;
  private double wristAngle;
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

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void resetWrist(double encoderValue) {
    wristRotation.getSensorCollection().setQuadraturePosition((int) encoderValue, RobotConstants.ARM_RESET_TIMEOUTMS);
  }

  /**
   * Setup for movement and Motion Magic
   */
  public void moveWrist(double wristAngle) {  
		wristRotation.configMotionCruiseVelocity(15000, kTimeoutMs);
		wristRotation.configMotionAcceleration(6000, kTimeoutMs);
    wristRotation.set(ControlMode.MotionMagic, wristAngle);
  }
}

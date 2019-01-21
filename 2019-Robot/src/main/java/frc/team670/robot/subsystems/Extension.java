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
 * Controls motors for motion of extension
 */
public class Extension extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  private TalonSRX extensionMotor;
  private double extensionLength;
  private static final double kF = 0, kP = 0, kI = 0, kD = 0; //TODO figure out what these are
  // Also need to add pull gains slots
  private static final int kPIDLoopIdx = 0, kSlotIdx = 0, kTimeoutMs = 0;

  public Extension() {
    extensionMotor = new TalonSRX(RobotMap.ARM_EXTENSION_MOTOR);
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
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

    /**
   * Setup for movement and Motion Magic
   */
  public void moveExtension(double extensionLength) {
    extensionMotor.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		extensionMotor.config_kF(kSlotIdx, kF, kTimeoutMs);
		extensionMotor.config_kP(kSlotIdx, kP, kTimeoutMs);
		extensionMotor.config_kI(kSlotIdx, kI, kTimeoutMs);
    extensionMotor.config_kD(kSlotIdx, kD, kTimeoutMs);
    // OPTIONAL: Set acceleration and vcruise velocity
		//extensionMotor.configMotionCruiseVelocity(15000, kTimeoutMs);
		//extensionMotor.configMotionAcceleration(6000, kTimeoutMs);
    extensionMotor.set(ControlMode.MotionMagic, extensionLength);
  }
}
